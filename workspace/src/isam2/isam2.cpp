// #include "isam2.hpp"
#include <type_traits>
#include <bits/stdc++.h>
// Camera observations of landmarks will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <Eigen/Dense>
// Each variable in the system (sposes and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem
// incrementally, so include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set of new factors to be added stored in a factor
// graph, and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use Projection factors to model the
// camera's landmark observations. Also, we will initialize the robot at some
// location using a Prior factor.
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

#include <vector>
#include <tuple>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <iostream>
#include <fstream>
using namespace std;
using namespace gtsam;
using namespace std::chrono;

static const int NUM_THREADS = 12;

/* Minimum m_dist length before paralellizing */
static const int MIN_M_DIST = NUM_THREADS;
static thread all_threads[NUM_THREADS];
static condition_variable cv;
static condition_variable step_cv;

// static const double M_DIST_TH = 85;
// static const double M_DIST_TH = 0.48999999463; //real data 0.01, 0.01, 0.5 LandmarkNoiseModel
static const double M_DIST_TH_STRAIGHTS = 85;
// static const double M_DIST_TH_TURNS = 500;
// static const double M_DIST_TH_TURNS = 0.089999; //could work really well for 0.1, 0.1, 0.28; dyaw = 1.35
static const double M_DIST_TH_HI = 40;
static const double M_DIST_TH_lLO = 20;

static const double TURNING_CONSTANT_LOW = 0.10;
static const double TURNING_CONSTANT_HI = 0.30;
static const 
// static const double M_DIST_TH = 4.5;

// static const double M_DIST_TH = 45; // used to be 45 lmao
static const long SEC_TO_NANOSEC = 1000000000;
static const int HEURISTIC_N = 10;
static vector<int> blue_cone_IDs;
static vector<int> yellow_cone_IDs;

static vector<tuple<void*, char>> work_queue;
static mutex work_queue_mutex;
static mutex isam2_mutex;
static mutex assoc_wait_mutex;
static mutex minID_wait_mutex;
static mutex step_wait_mutex;
static atomic<int> assoc_counter(0);
static atomic<int> minID_counter(0);

atomic<bool> prev_DA_done;



// static const float DT = 0.1;
// static const float SIM_TIME = 50.0;
// static const int LM_SIZE = 2;
// static const int STATE_SIZE = 3;
// static const int N_STEP = 100;

class Assoc_Args
{
    public:

    vector<Point2> *cone_obs_blue;
    vector<Point2> *cone_obs_yellow;


    int lo;
    int hi;


    /* cone_obs must be a pointer be thread_safe */
    Assoc_Args(vector<Point2> *cone_obs_blue, vector<Point2> *cone_obs_yellow, int lo, int hi)
    {
        /* separate because when you data associate and fine a new cone,
         * you want to know the color of the new cone you add */
        this->cone_obs_blue = cone_obs_blue;
        this->cone_obs_yellow = cone_obs_yellow;

        /* cannot separate all_cone_est because access to marginal cov is also like this */
        /* Not necessarily: the index in color_cone_est indicates the index in color_cone_IDs to access*/



        this->lo = lo;
        this->hi = hi;

    }
};

class MinID_Args
{
    public:
    int lo;
    int hi;

    bool is_blue_obs;
    bool is_yellow_obs;

    Point2 obs;
    int prev_color_n_landmarks;

    
    vector<Pose2> *blue_glob_obs;
    vector<Pose2> *yellow_glob_obs;

    int num_obs;

    Pose2 glob_pos_bearing;


    MinID_Args (bool is_blue_obs, bool is_yellow_obs, Point2 obs,
                int prev_color_n_landmarks, 
                vector<Pose2> *blue_glob_obs, vector<Pose2> *yellow_glob_obs,
                int num_obs, Pose2 glob_pos_bearing, int lo, int hi)
    {
        this->is_blue_obs = is_blue_obs;
        this->is_yellow_obs = is_yellow_obs;

        this->obs = obs;

        this->prev_color_n_landmarks = prev_color_n_landmarks;

        this->blue_glob_obs = blue_glob_obs;
        this->yellow_glob_obs = yellow_glob_obs;

        this->num_obs = num_obs;

        this->glob_pos_bearing = glob_pos_bearing;

        this->lo = lo;
        this->hi = hi;

    }

};




class slamISAM {

private: ISAM2Params parameters;
    ISAM2 isam2;
    //Create a factor graph and values for new data
    NonlinearFactorGraph graph;
    Values values;

    int pose_num;
    bool first_pose_added;

    gtsam::Symbol X(int robot_pose_id) {
        return Symbol('x', robot_pose_id);
    }

    gtsam::Symbol L(int cone_pose_id) {
        return Symbol('l', cone_pose_id);
    }

    /* Assoc_Args common arguments */
    vector<Point2> cone_obs_blue;
    vector<Point2> cone_obs_yellow;

    Eigen::MatrixXd blue_global_cone_x;
    Eigen::MatrixXd blue_global_cone_y;

    Eigen::MatrixXd yellow_global_cone_x;
    Eigen::MatrixXd yellow_global_cone_y;

    Eigen::MatrixXd blue_bearing;
    Eigen::MatrixXd yellow_bearing;

    vector<Pose2> blue_cone_est;
    vector<Pose2> yellow_cone_est;

    Pose2 global_odom;
    
    vector<double> m_dist;


public:
    high_resolution_clock::time_point start;
    int n_landmarks;
    int blue_n_landmarks;
    int yellow_n_landmarks;

    bool heuristic_run;
    
    //gtsam::Pose2 robot_est;
    std::vector<gtsam::Pose2> landmark_est;
    std::vector<Point2> orange_cones;

    /* how the landmark estimates are organized */
    vector<int> blue_cone_IDs;
    vector<int> yellow_cone_IDs;

    Vector LandmarkNoiseModel;
    noiseModel::Diagonal::shared_ptr landmark_model;
    Vector PriorNoiseModel;
    noiseModel::Diagonal::shared_ptr prior_model;
    Vector OdomNoiseModel;
    noiseModel::Diagonal::shared_ptr odom_model;

    double m_dist_th;

    slamISAM(rclcpp::Logger logger) {
        parameters = ISAM2Params(ISAM2DoglegParams(),0.1,10,true);
        parameters.setFactorization("QR");

        isam2 = gtsam::ISAM2(parameters);
        graph = gtsam::NonlinearFactorGraph();
        values = gtsam::Values();
        pose_num = 0;
        first_pose_added = false;
        n_landmarks = 0;
        //robot_est = gtsam::Pose2(0, 0, 0);
        landmark_est = std::vector<gtsam::Pose2>();

        orange_cones = std::vector<Point2>();
        for (int i = 0; i < NUM_THREADS; i++)
        {
            all_threads[i] = thread(&slamISAM::process_work_queue, this, logger, i);
        }

        heuristic_run = true;
        prev_DA_done = true;

        LandmarkNoiseModel = Vector(3);
        // used to be 0.01 for real data
        // 0 for EUFS_SIM
        //TODO: have a different noise model at the beginning
        LandmarkNoiseModel(0) = 0.3;
        LandmarkNoiseModel(1) = 0.3;
        LandmarkNoiseModel(2) = 0.3;
        landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);

        // used to be all 0s for EUFS_SIM
        PriorNoiseModel = Vector(3);
        PriorNoiseModel(0) = 0.3;
        PriorNoiseModel(1) = 0.3;
        PriorNoiseModel(2) = 0.3;
        prior_model = noiseModel::Diagonal::Sigmas(PriorNoiseModel);

/* Go from 1 pose to another pose*/
        OdomNoiseModel = Vector(3);
        OdomNoiseModel(0) = 0.3;
        OdomNoiseModel(1) = 0.3;
        OdomNoiseModel(2) = 0.3; 
        odom_model = noiseModel::Diagonal::Sigmas(OdomNoiseModel);



        m_dist_th = M_DIST_TH_STRAIGHTS;
    }




    /**
     * @brief Processes Associate tasks (indicated by Assoc_Arg).
     *        Given a lower index (lo) and upper index (hi) of
     *        m_dist, calculate the mahalanobis distances of
     *        all elements between those indices. This involves
     *        knowing which observed cone the index corresponds to.
     *
     */
    void associate (rclcpp::Logger logger, Assoc_Args *A_task)
    {
        
        // auto landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);



        int num_obs_blue = (int)A_task->cone_obs_blue->size();
        int num_obs_yellow = (int)A_task->cone_obs_yellow->size();
        int num_obs = num_obs_blue + num_obs_yellow;
        if (blue_n_landmarks == 0 && yellow_n_landmarks == 0) /*no previously seen cones */
        {
            /* all obs cones are new */
            isam2_mutex.lock();
            for (int i = 0; i < num_obs_blue; i++)
            {

                graph.add(BetweenFactor<Pose2>(X(pose_num), L(n_landmarks),
                            Pose2(A_task->cone_obs_blue->at(i).x(),
                                    A_task->cone_obs_blue->at(i).y(),
                                    blue_bearing(i, 0)), landmark_model));

                values.insert(L(n_landmarks), Pose2(blue_global_cone_x(i),
                                                    blue_global_cone_y(i),
                                                        0));

		        
                if (n_landmarks == 0)
                {
                    RCLCPP_INFO(logger, "first landmark added");
                }

	    	    

                blue_cone_IDs.push_back(n_landmarks);
                blue_n_landmarks++;
                n_landmarks++;

            }

            for (int i = 0; i < num_obs_yellow; i++)
            {
                graph.add(BetweenFactor<Pose2>(X(pose_num), L(n_landmarks),
                            Pose2(A_task->cone_obs_yellow->at(i).x(),
                                    A_task->cone_obs_yellow->at(i).y(),
                                    yellow_bearing(i, 0)), landmark_model));

                 values.insert(L(n_landmarks), Pose2(yellow_global_cone_x(i),
                                                    yellow_global_cone_y(i),
                                                        0));
		        
                if (n_landmarks == 0)
                {
                    RCLCPP_INFO(logger, "first landmark added");
                }

		        

                yellow_cone_IDs.push_back(n_landmarks);
                yellow_n_landmarks++;
                n_landmarks++;
            }


            isam2.update(graph, values);
            isam2.update();
            values.clear();
            graph.resize(0);
            prev_DA_done = true;
            //pose_num++;
            step_cv.notify_one();

            isam2_mutex.unlock();

            RCLCPP_INFO(logger, "completed first DA cycle");
            return;


        }

        
        /* For each blue observed cone, calculate Mahalanobis distance wrt blue_n_landmarks
         * The +1 represents the M_DIST_TH for that observation
         *
         * Likewise for each yellow observed cone
         * How many previous landmarks are you looking at?
         */
         
        int blue_multiple_size = blue_n_landmarks + 1;
        int yellow_multiple_size = yellow_n_landmarks + 1;
        bool using_heuristic_n = false;
        /* Use when there's many previous estimates */
        /* TODO: <= ?????*/
        if (heuristic_run && HEURISTIC_N < blue_n_landmarks &&
                            HEURISTIC_N < yellow_n_landmarks)
        {
            blue_multiple_size = HEURISTIC_N + 1;
            yellow_multiple_size = HEURISTIC_N + 1;
            using_heuristic_n = true;
        }

        int first_yellow_idx = num_obs_blue * blue_multiple_size;
        int i = A_task->lo;
        bool cur_obs_is_blue = (i < first_yellow_idx);

        /**
         * Determine which observation landmark to calculate the mahal. dist. wrt
         * if lo < first_yellow_idx, then obs_id should start at a blue cone
         */

        vector<Pose2> *color_cone_est;
        int ith_color_cone;
        int obs_id;
        if (cur_obs_is_blue)
        {
            /* for indexing into cone_obs_blue */
            color_cone_est = &blue_cone_est;
            obs_id = (int)(A_task->lo / blue_multiple_size);
            ith_color_cone = A_task->lo % blue_multiple_size;
        }
        else if (!cur_obs_is_blue)
        {
            /* for indexing into cone_obs_yelow */
            color_cone_est = &yellow_cone_est;
            int offset_start = (int)(A_task->lo - first_yellow_idx);
            obs_id = (int)(offset_start / yellow_multiple_size);
            ith_color_cone = offset_start % yellow_multiple_size;
        }
        
        /*not necessary because the initialization time was 0*/
        // auto end_t_assoc = high_resolution_clock::now();
        // auto dur_t_assoc = duration_cast<microseconds>(end_t_assoc - start_t_assoc);
        // RCLCPP_INFO(logger, "thread assoc_args init [lo=%d, hi=%d) : time %d",
        //                     A_task->lo, A_task->hi, dur_t_assoc.count());
        
        while (i < A_task->hi)
        {
            int last_idx_for_cur_obs;
            vector<int> *cone_IDs;

            Eigen::MatrixXd *global_cone_x;
            Eigen::MatrixXd *global_cone_y;

            /* last_idx_for_cur_obs: the last index in m_dist to calculate up to
             *                       for the current observation
             *
             * obs_id: the index of either blue or yellow observed cone
             *         Whether the index represents blue or yellow depends on
             *         which index of m_dist you are calculating
             *
             *         obs_id in [0, cone_obs_blue->size())
             *         obs_id in [0, cone_obs_yellow->size())
             *
             * cone_IDs: a pointer to either blue_cone_IDs or yellow_cone_IDs
             *           which holds the landmark_IDs of the cones, separated
             *           by color
             */

            /* TODO: SUSPECT 1 
             * is global_cone_x and global_cone_y initialized properly?
             * 
             * Chances are yes, they most likely are because you aren't getting
             * any index out of bounds error;
             */
        

            if (cur_obs_is_blue) /* blue observation */
            {
                last_idx_for_cur_obs = ((obs_id + 1) *
                                            blue_multiple_size - 1);
                cone_IDs = &blue_cone_IDs;
                global_cone_x = &blue_global_cone_x;
                global_cone_y = &blue_global_cone_y;

            }
            else if (!cur_obs_is_blue) /* yellow observations */
            {
                last_idx_for_cur_obs = (first_yellow_idx +
                                        ((obs_id + 1) *
                                         yellow_multiple_size) - 1);
                cone_IDs = &yellow_cone_IDs;
                global_cone_x = &yellow_global_cone_x;
                global_cone_y = &yellow_global_cone_y;
            }

            /* calculate mahalanobis distance for all previous cones wrt to
             * current obs cone */
            
            /*RCLCPP_INFO(logger, "lo: %d | hi: %d | blue_n_landmarks: %d | yellow_n_landmarks: %d"
                                "first_yellow_idx: %d", A_task->lo, A_task->hi, blue_n_landmarks,
                                yellow_n_landmarks, first_yellow_idx); */

            int offset_id_idx = 0;
            if (using_heuristic_n)
            {
                if (cur_obs_is_blue)
                {
                    offset_id_idx = blue_n_landmarks - HEURISTIC_N;
                }
                else
                {
                    offset_id_idx = yellow_n_landmarks - HEURISTIC_N;
                }
            }
            
            for (; i < last_idx_for_cur_obs && i < A_task->hi; i++)
            {

                Eigen::MatrixXd diff(1, 3);

                //assert(cone_IDs->at(ith_color_cone) < (int)A_task->all_cone_est->size());
                

		        
                Pose2 prev_est = color_cone_est->at(ith_color_cone);
                diff << (*global_cone_x)(obs_id, 0) - prev_est.x(),
                        (*global_cone_y)(obs_id, 0) - prev_est.y(),
                        1;
                int id = cone_IDs->at(ith_color_cone + offset_id_idx);
                // auto start_t_assoc = high_resolution_clock::now();
                m_dist.at(i) = (diff * isam2.marginalCovariance(L(id)) * diff.transpose())(0, 0);
                /* isam2.marginalCovariance(L(id)) takes a long time to obtain*/
                // m_dist.at(i) = (diff * diff.transpose())(0, 0);
                // auto end_t_assoc = high_resolution_clock::now();
                // auto dur_t_assoc = duration_cast<microseconds>(end_t_assoc - start_t_assoc);
                // RCLCPP_INFO(logger, "for loop thread assoc_args [lo=%d, hi=%d) : time %d",
                //                 A_task->lo, A_task->hi, dur_t_assoc.count());
                
                
                ith_color_cone++;

            }
            

            if (i == A_task->hi)
            {
                break;
            }
            /* TODO: Suspect 2 
             * The M_DIST_TH are not in the right places: 
             * They shouldn't be at the beginning, they should be at the end of each multiple
             *
             * Is last_idx_for_cur_obs calculated properly? 
             * Result: It is
             */
            m_dist.at(last_idx_for_cur_obs) = m_dist_th;
            //RCLCPP_INFO(logger, "last_idx_for_cur_obs: %d", last_idx_for_cur_obs);
            i++; /* skip 1 element (reserved for M_DIST_TH */
            obs_id++;

            if (obs_id == num_obs_blue && cur_obs_is_blue)
            {
                obs_id = 0;
                cur_obs_is_blue = false;
                color_cone_est = &yellow_cone_est;
            }
            ith_color_cone = 0;
        }

        /* atomic operation*/
        /* TODO: thread A could be sitting at NUM_THREADS - 1
         * But then it may not have checked the if statement conditional yet
         * 
         * In the meantime, thread B may increment assoc_counter and then
         * both check the conditional at the same time*/
        // auto end_t_assoc = high_resolution_clock::now();
        // auto dur_t_assoc = duration_cast<microseconds>(end_t_assoc - start_t_assoc);
        // RCLCPP_INFO(logger, "thread assoc_args [lo=%d, hi=%d) : time %d",
        //                     A_task->lo, A_task->hi, dur_t_assoc.count());
        assoc_counter++;
        if ((assoc_counter.load() == NUM_THREADS) ||
                ((m_dist.size() < MIN_M_DIST) && (assoc_counter.load() == 1)))
        {
            cv.notify_one();
        }






        /* Wait for all other Assoc_Args
         * to finish
         *
         * Then start MinID_Args tasks
         *
         */

        if (A_task->lo == 0)
        {

            unique_lock<mutex> assoc_lk(assoc_wait_mutex);
            RCLCPP_INFO(logger, "waiting for Assoc_Args");
            int m_dist_size = (int)m_dist.size();
            cv.wait(assoc_lk, [m_dist_size]{return ((assoc_counter.load() == NUM_THREADS) ||
                                ((m_dist_size < MIN_M_DIST) && (assoc_counter.load() == 1)));});

            RCLCPP_INFO(logger, "finished all Assoc_Args");
            auto end_m_dist = high_resolution_clock::now();
            auto dur = duration_cast<microseconds>(end_m_dist - start);
            RCLCPP_INFO(logger, "m_dist calc time: %d", dur.count());

            /*waiting until all Assoc_Args tasks have finished*/
            int num_obs = (num_obs_blue + num_obs_yellow);

            

            assoc_counter = 0;

            /* add MinIDs_Args to the work queue */
            int lo = 0;
            vector<Pose2> *blue_glob_obs = NULL;
            vector<Pose2> *yellow_glob_obs = NULL;
	        if (heuristic_run)
	        {
                blue_glob_obs = new vector<Pose2>;
                yellow_glob_obs = new vector<Pose2>;
	        }


            work_queue_mutex.lock();
            RCLCPP_INFO(logger, "num_obs_blue: %d | num_obs_yellow: %d",
                                num_obs_blue, num_obs_yellow);
            /* Remember: For blue_multiple_size and yellow_multiple_size to both 
             * both be HEURISTIC_N + 1, HEURISTIC_N must be less than both 
             * blue_n_landmarks AND yellow_n_landmarks
             * 
             * TODO: consider changing this in the future?
             */
            for (int i = 0; i < num_obs_blue; i++)
            {
                int hi = lo + blue_multiple_size;
                Pose2 glob_pos_bearing = Pose2(blue_global_cone_x(i, 0),
                                                blue_global_cone_y(i, 0),
                                                blue_bearing(i, 0));


                
                MinID_Args *M_task = new MinID_Args(true, false, this->cone_obs_blue.at(i),
                                                    (int)blue_cone_est.size(),
                                        blue_glob_obs, yellow_glob_obs,
                                                    num_obs, glob_pos_bearing, lo, hi);
                RCLCPP_INFO(logger, "blue id: %d | minID: [%d, %d)", i, lo, hi);
                lo = hi;
                work_queue.push_back(make_tuple(M_task, 'm'));
            }

            for (int i = 0; i < num_obs_yellow; i++)
            {
                int hi = lo + yellow_multiple_size;

                Pose2 glob_pos_bearing = Pose2(yellow_global_cone_x(i, 0),
                                                yellow_global_cone_y(i, 0),
                                                yellow_bearing(i, 0));
                MinID_Args *M_task = new MinID_Args(false, true, this->cone_obs_yellow.at(i),
                                                    (int)yellow_cone_est.size(),
                                       blue_glob_obs, yellow_glob_obs,
                                                    num_obs, glob_pos_bearing, lo, hi);
                RCLCPP_INFO(logger, "yellow id: %d | minID: [%d, %d)", i, lo, hi);
                lo = hi;
                work_queue.push_back(make_tuple(M_task, 'm'));
            }

            cone_obs_blue.clear();
            cone_obs_yellow.clear();
            /* when data associating unknown observations, blue_cone_obs and yellow_cone_obs 
             * must hold these unknown obs*/

            work_queue_mutex.unlock();
            RCLCPP_INFO(logger, "Last Assoc_Arg added MinID_Args to queue: m_dist_len: %d",
                                    (int)m_dist.size());
            
            
            

        }
        delete A_task;

    }

    void find_minIDs(rclcpp::Logger logger, MinID_Args *M_task)
    {
        // Vector NoiseModel(3);
        // NoiseModel(0) = 0;
        // NoiseModel(1) = 0;
        // NoiseModel(2) = 0;

        
        //print_cones(logger, cone_obs);
        // static auto landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);

        /* identify the min ID */
        /*
            int start_offset = i * (n_landmarks + 1);
            vector<float>::iterator start_iter = m_dist->begin() + start_offset;
            min_ids->at(i) = std::distance(start_iter,
                                        std::min_element(start_iter,
                                            start_iter + (n_landmarks+1)));
        */



        int num_obs = M_task->num_obs;
        vector<double>::iterator start_iter = m_dist.begin() + M_task->lo;
        vector<double>::iterator end_iter = m_dist.begin() + M_task->hi;

        int minID = std::distance(start_iter,
                                std::min_element(start_iter, end_iter));
	    
        /* Identify if the min ID is actual M_DIST_TH
         * Add the relationship to the graph
         * - Only 1 new cone can be added to graph at a time
         *
         * For new landmarks: you have to add it as a new value
         * Then you have to determine their overall landmark id
         * Add the id to the correct color_cone_ID vector
         * Increment the color_n_landmarks counter
         *
         * What to do if heuristic_run = true and is a new landmark?
         * - Add to M_task->unknown_obs
         *
         * Re-add as Assoc_Args to work queue for cones that are
         * registered as new cones during heuristic_run
         */
        RCLCPP_INFO(logger, "minID: %d | dist: %lf | lo: %d, hi: %d", 
                    minID, *(m_dist.begin() + M_task->lo + minID), 
                    M_task->lo, M_task->hi);
        isam2_mutex.lock();

        /* updating the iSAM2 model by determining if cone is new or not */

        /* prev_color_n_landmarks = color_n_landmarks if not using HEURISTIC_N
         * prev_color_n_landmarks = HEURISTIC_N otherwise
         */
        if (minID == M_task->prev_color_n_landmarks) /* new landmark */
        {
            if (heuristic_run)
            {
                if (M_task->is_blue_obs && !(M_task->is_yellow_obs)) /*is blue*/
                {
                    cone_obs_blue.push_back(M_task->obs);
                    M_task->blue_glob_obs->push_back(M_task->glob_pos_bearing);
		            // RCLCPP_INFO(logger, "blue_unknown_obs size: %d", 
				    //                     (int)cone_obs_blue.size());
                }
                else if (M_task->is_yellow_obs && !(M_task->is_blue_obs)) /*is yellow*/
                {
                    cone_obs_yellow.push_back(M_task->obs);
                    M_task->yellow_glob_obs->push_back(M_task->glob_pos_bearing);
                    // RCLCPP_INFO(logger, "yellow_unknown_obs size: %d", 
                    //                     (int)cone_obs_yellow.size());
                }
            }
            else /* not heuristic_run => add to isam2 */
            {
                graph.add(BetweenFactor<Pose2>(X(pose_num), L(n_landmarks),
                                        Pose2(M_task->obs.x(), M_task->obs.y(),
                                            M_task->glob_pos_bearing.theta()),
                                            landmark_model));

                values.insert(L(n_landmarks), Pose2(M_task->glob_pos_bearing.x(),
                                                    M_task->glob_pos_bearing.y(),
                                                    0));

                if (M_task->is_blue_obs && !(M_task->is_yellow_obs))
                {
                    blue_cone_IDs.push_back(n_landmarks);
                    blue_n_landmarks++;
                }
                else if (M_task->is_yellow_obs && !(M_task->is_blue_obs))
                {
                    yellow_cone_IDs.push_back(n_landmarks);
                    yellow_n_landmarks++;
                }
                n_landmarks++;

            }
        }
        else /* old landmark */
        {
            /* doesn't matter if heuristic_run or not */
            /* RCLCPP_INFO(logger, "old cone; color id: %d",
                                M_task->color_cone_IDs->at(minID)); */
            /*if (M_task->is_blue_obs)
            {
                RCLCPP_INFO(logger, "minID: %d old cone; blue_cone_IDs size: %d vs %d",
                                minID, (int)blue_cone_IDs.size(),
                                (int)M_task->color_cone_IDs->size());
            }
            else
            {
                RCLCPP_INFO(logger, "minID: %d old cone; yellow_cone_ids size: %d",
                                minID, (int)yellow_cone_IDs.size());
            } */


            int temp_id = -1;
            if (!heuristic_run || HEURISTIC_N >= blue_n_landmarks || HEURISTIC_N >= yellow_n_landmarks)
            {
                /* not using HEURISTIC_N */
                temp_id = minID;
            }
            else
            {
                /* using HEURISTIC_N */
                temp_id = M_task->prev_color_n_landmarks - HEURISTIC_N + minID;
            }

            int id = -1;
            if (M_task->is_blue_obs)
            {
                id = blue_cone_IDs.at(temp_id);
            }
            else if(M_task->is_yellow_obs)
            {
                id = yellow_cone_IDs.at(temp_id);
            }



            graph.add(BetweenFactor<Pose2>(X(pose_num), L(id),
                                        Pose2(M_task->obs.x(),
                                            M_task->obs.y(),
                                            M_task->glob_pos_bearing.theta()),
                                            landmark_model));

        }


        ///////////////////////////////////////////////////////////////////////
        // isam2.update(graph, values);
	    // isam2.update();
        // graph.resize(0);
        // values.clear();

        /* atomic operation */
        /* TODO: problem; 1 thread could increment to num_obs but another thread 
         * has yet to process the conditional in the if statement
         *
         * 2 threads will enter the if statement*/
        minID_counter++;
        if (minID_counter.load() == num_obs)
        {
            cv.notify_one();
        }


        /* Is calling cv.wait for a thread really necessary?
         * No: if minID_counter == num_obs and if only 1 thread can be updating
         * isam2 at a time, then that means all MinID_Args are finished
         * when they are processed 1 at a time
         */


        isam2_mutex.unlock();
        /* wait for all MinID_Args to be finished */
        /* Take the first MinID_Args; No way to actually get the last by color
         * You may not always have blue cones; you may not always have yellow cones*/
        if (M_task->lo == 0) /* pick 1 MinID_Args tasks */
        {
            
            unique_lock<mutex> minID_lk(minID_wait_mutex);
            cv.wait(minID_lk, [num_obs]{return (minID_counter.load() == num_obs);});
            
            minID_counter = 0;

            isam2.update(graph, values);
            isam2.update();
            graph.resize(0);
            values.clear();
            RCLCPP_INFO(logger, "finished processing last minID | is heuristic_run: %d", heuristic_run);

            int blue_num_u_obs = 0;
            int yellow_num_u_obs = 0;
            int num_u_obs = 0;

            if (heuristic_run)
            {
                blue_num_u_obs = (int)cone_obs_blue.size();
                yellow_num_u_obs = (int)cone_obs_yellow.size();
                num_u_obs = blue_num_u_obs + yellow_num_u_obs;

		RCLCPP_INFO(logger, "getting unknown observations; blue_num_u_obs: %d | yellow_num_u_obs: %d",
						blue_num_u_obs, yellow_num_u_obs);
            }
	    RCLCPP_INFO(logger, "num unknown observations: %d", num_u_obs);



            if (heuristic_run && num_u_obs > 0)
            {
                heuristic_run = false;
		        RCLCPP_INFO(logger, "processing unknown observations"); 

                /* create the new global_cone x and y and bearing */
                blue_bearing = Eigen::MatrixXd(blue_num_u_obs, 1);
                yellow_bearing = Eigen::MatrixXd(yellow_num_u_obs, 1);

                blue_global_cone_x = Eigen::MatrixXd(blue_num_u_obs, 1);
                blue_global_cone_y = Eigen::MatrixXd(blue_num_u_obs, 1);

                yellow_global_cone_x = Eigen::MatrixXd(yellow_num_u_obs, 1);
                yellow_global_cone_y = Eigen::MatrixXd(yellow_num_u_obs, 1);

                for (int i = 0; i < blue_num_u_obs; i++)
                {
                    blue_global_cone_x(i) = M_task->blue_glob_obs->at(i).x();
                }
                for (int i = 0; i < blue_num_u_obs; i++)
                {
                    blue_global_cone_y(i) = M_task->blue_glob_obs->at(i).y();
                }
                for (int i = 0; i < blue_num_u_obs; i++)
                {
                    blue_bearing(i) = M_task->blue_glob_obs->at(i).theta();
                }


                for (int i = 0; i < yellow_num_u_obs; i++)
                {
                    yellow_global_cone_x(i) = M_task->yellow_glob_obs->at(i).x();
                }
                for (int i = 0; i < yellow_num_u_obs; i++)
                {
                    yellow_global_cone_y(i) = M_task->yellow_glob_obs->at(i).y();
                }
                for (int i = 0; i < yellow_num_u_obs; i++)
                {
                    yellow_bearing(i) = M_task->yellow_glob_obs->at(i).theta();
                }

                blue_cone_est.clear();
                yellow_cone_est.clear();

                for (int i = 0; i < blue_n_landmarks; i++)
                {
                    int id = blue_cone_IDs.at(i);
                    blue_cone_est.push_back(isam2.calculateEstimate(L(id)).cast<Pose2>());
                }

                for (int i = 0; i < yellow_n_landmarks; i++)
                {
                    int id = yellow_cone_IDs.at(i);
                    yellow_cone_est.push_back(isam2.calculateEstimate(L(id)).cast<Pose2>());
                }

                /* Add Assoc_Args for unknown_obs */
                int m_dist_len = ((blue_n_landmarks + 1) * blue_num_u_obs +
                            (yellow_n_landmarks + 1) * yellow_num_u_obs);
                m_dist.resize(m_dist_len);
                int multiple_size = (int)(m_dist_len / NUM_THREADS);
                int remainders = m_dist_len - (NUM_THREADS * multiple_size);

                if (m_dist_len < MIN_M_DIST)
                {
                    multiple_size = m_dist_len;
                    remainders = 0;
                }

                int lo = 0;
                int hi = 0;

                work_queue_mutex.lock();
                while (lo < m_dist_len && hi < m_dist_len)
                {
                    hi = lo + multiple_size;

                    if (remainders > 0)
                    {
                        hi++;
                        remainders--;
                    }
                    
                    Assoc_Args *A_task = new Assoc_Args(&(this->cone_obs_blue), &(this->cone_obs_yellow), lo, hi);
                    work_queue.push_back(make_tuple(A_task, 'a'));

                    lo = hi;

                }
                work_queue_mutex.unlock();

                /* No Assoc_Args may be added if all observations are of
                 * old cones
                 */
                RCLCPP_INFO(logger, "wrapping up heuristic run");

            }
            else /* minIDs results represent the final results */
            {
                /*reset heuristic_run to true to prep for next cycle*/
                heuristic_run = true;
                prev_DA_done = true;
                //pose_num++;
                

                step_cv.notify_one();

                RCLCPP_INFO(logger, "finished prev DA | m_dist_th: %lf", m_dist_th);
            }



        }


        /* if heuristic_run, after filling up the unknown_obs, then you have to
         * do data association again
         */
         delete M_task;
    }


    /**
     * @brief Callback function for threads. Runs infinitely because threads
     *        persist while SLAM is running. Is responsible for data
     *        association and updating the iSAM2 model. Step will add tasks
     *        to the work queue, pm_cov.atrocess_work_queue will process those tasks.
     */
    void process_work_queue(rclcpp::Logger logger, int t_id)
    {
        bool locked = false;

        while (true)
        {
            work_queue_mutex.lock();
            locked = true;
            if (work_queue.size() != 0)
            {

                /* When Assoc_Args are properly added, are we properly
                 * processing them?
                 *
                 * TODO: NO
                 */
                //assert('i' == 'z');
                // RCLCPP_INFO(logger, "work queue size = %d; t_id: %d",
                //                     (int)work_queue.size(), t_id);
                /* retrieve and remove oldest task from the queue */
                tuple<void*, char> unknown_task = work_queue.at(0);
                work_queue.erase(work_queue.begin());
                work_queue_mutex.unlock();
                locked = false;

                if (get<1>(unknown_task) == 'a') /* task = data assoc */
                {
                    Assoc_Args *A_task = (Assoc_Args*) get<0>(unknown_task);
                    associate(logger, A_task);
                }
                else if (get<1>(unknown_task) == 'm') /* task = minID */
                {

                    MinID_Args *M_task = (MinID_Args*) get<0>(unknown_task);
                    find_minIDs(logger, M_task);
                }
            }
            //else
            //{
            //    RCLCPP_INFO(logger, "empty work queue");
            //}

            if (locked)
            {
                work_queue_mutex.unlock();
            }
        }
    }

    /**
     * @brief Obtains information about the observed cones from the current time
     *        step as well as odometry information (to use motion model to
     *        calculate current pose). Will add data association tasks to the
     *        work queue.
     */
    void step(auto logger, gtsam::Pose2 global_odom, vector<Point2> &cone_obs,
                vector<Point2> &cone_obs_blue, vector<Point2> &cone_obs_yellow,
                vector<Point2> &orange_ref_cones, gtsam::Pose2 velocity,
                long time_ns, bool loopClosure)
    {
        
        if (prev_DA_done == false)
        {
                RCLCPP_INFO(logger, "waiting at step for prev DA");
        }
        unique_lock<mutex> step_lk(step_wait_mutex);

        
        /* Calculate the appropriate noise model settings 
         * dyaw in (0.1, 0.32)
         * */
        

        

        RCLCPP_INFO(logger, "\nx%d", pose_num);
        step_cv.wait(step_lk, []{return (prev_DA_done.load());});
        if (first_pose_added)
        {
            pose_num++;
        }

        if (n_landmarks > 0)
        {
            auto end = high_resolution_clock::now();
            auto d = duration_cast<microseconds>(end - start);
            RCLCPP_INFO(logger, "Data Association time: %d", d.count());
        }
        start = high_resolution_clock::now();
        prev_DA_done = false;

        

        if (pose_num==0) {//if this is the first pose, add your inital pose to the factor graph
            //std::cout << "First pose\n" << std::endl;
            
            gtsam::PriorFactor<Pose2> prior_factor = gtsam::PriorFactor<Pose2>(X(0), global_odom, prior_model);
            //add prior
            graph.add(prior_factor);
            values.insert(X(0), global_odom);

            first_pose_added = true;



            //ASSUMES THAT YOU SEE ORANGE CONES ON YOUR FIRST MEASUREMENT OF LANDMARKS
            //Add orange cone left and right
            //hopefully it's only 2 cones
            orange_cones = orange_ref_cones;
        }
        else {
            //std::cout << "New Pose\n" << std::endl;
            
            Pose2 prev_pos = isam2.calculateEstimate(X(pose_num - 1)).cast<Pose2>();
            //create a factor between current and previous robot pose
            //add odometry estimates
            //Motion model


            //TODO: change back to motion model with velocity
            double time_s = time_ns/SEC_TO_NANOSEC;
            
            // Pose2 Odometry =  Pose2(velocity.x()*time_s, velocity.y()*time_s, global_odom.theta() - prev_pos.theta());
            Pose2 Odometry = Pose2(velocity.x() * time_s, velocity.y()*time_s, velocity.theta()*time_s);
            /*real data motion model?
            Pose2 Odometry =  Pose2(global_odom.x() - prev_pos.x(),global_odom.y() - prev_pos.y(),
                                    global_odom.theta() - prev_pos.theta());
                                    */

            
            

            gtsam::BetweenFactor<Pose2> odom_factor = gtsam::BetweenFactor<Pose2>(X(pose_num - 1), X(pose_num),
                                                                                    Odometry, odom_model);
            graph.add(odom_factor);
            values.insert(X(pose_num), global_odom);
        }
        
        

        

        RCLCPP_INFO(logger, "beginning loop closure");
        if (loopClosure) {
            // std::cout<<"loop closure constraint added"<<std::endl;
            static noiseModel::Diagonal::shared_ptr loop_closure_model = noiseModel::Diagonal::Sigmas(OdomNoiseModel);
            //left is 0, right is 1
            //Do triangulation given the current cone positions and the previous saved orange cones
            //take the orange cones, look at difference between
            //the left and right cones
            Point2 diffLeft = orange_cones[0] - orange_ref_cones[0];
            Point2 diffRight = orange_cones[1] - orange_ref_cones[1];
            Point2 avgDiff = (diffLeft+diffRight)/2; //pose difference between first pose and last pose
            double angle = atan2(avgDiff.y(), avgDiff.x());
            Pose2 AvgPoseDiff = Pose2(avgDiff.x(), avgDiff.y(),angle);
            // std::cout<<"loop closure pose diff:"<<AvgPoseDiff<<std::endl;

            gtsam::BetweenFactor<Pose2> odom_factor = gtsam::BetweenFactor<Pose2>(X(0), X(pose_num),AvgPoseDiff, loop_closure_model);
            graph.add(odom_factor);
            values.insert(X(pose_num), global_odom);
        }
        RCLCPP_INFO(logger, "end loop closure");

        //todo only do this once after update
        isam2.update(graph, values);
	    isam2.update();
        graph.resize(0);
        values.clear();
        
        /* BEGIN VECTORIZATION*/
        
        /* vectorizations to calculate the global x and y positions of the cones*/
	    Eigen::MatrixXd range(cone_obs.size(), 1);
	    Eigen::MatrixXd bearing(cone_obs.size(), 1);

	    int num_obs = (int)cone_obs.size();

	    for (int i = 0; i < num_obs; i++)
	    {
	        range(i) = norm2(cone_obs.at(i));
	    }

	    for (int i = 0; i < num_obs; i++)
	    {
	        bearing(i) = atan2(cone_obs.at(i).y(), cone_obs.at(i).x());
	    }

	    Eigen::MatrixXd totalBearing = bearing.array()+global_odom.theta();

        /* do this twice, once for yellow obs and once for blue obs */
	    Eigen::MatrixXd global_cone_x = Eigen::MatrixXd(cone_obs.size(),1);
	    Eigen::MatrixXd global_cone_y = Eigen::MatrixXd(cone_obs.size(),1);

	    Eigen::MatrixXd totalBearing_cos(cone_obs.size(),1);
	    Eigen::MatrixXd totalBearing_sin(cone_obs.size(),1);
	    totalBearing_cos = totalBearing.array().cos();
	    totalBearing_sin = totalBearing.array().sin();

	    global_cone_x = global_odom.x() + range.array()*totalBearing_cos.array();
        global_cone_y = global_odom.y() + range.array()*totalBearing_sin.array();


        int num_obs_blue = (int)cone_obs_blue.size();
        int num_obs_yellow = (int)cone_obs_yellow.size();


        blue_global_cone_x = global_cone_x.block(0, 0,
                                                    num_obs_blue, 1);
        blue_global_cone_y = global_cone_y.block(0, 0,
                                                    num_obs_blue, 1);

        yellow_global_cone_x = global_cone_x.block(num_obs_blue, 0,
                                                    num_obs_yellow, 1);

        yellow_global_cone_y = global_cone_y.block(num_obs_blue, 0,
                                                    num_obs_yellow, 1);


        blue_bearing = Eigen::MatrixXd(num_obs_blue, 1);
        yellow_bearing = Eigen::MatrixXd(num_obs_yellow, 1);

        for (int i = 0; i < num_obs_blue; i++)
        {
            blue_bearing(i) = bearing(i);
        }

        for (int i = 0; i < num_obs_yellow; i++)
        {
            yellow_bearing(i) = bearing(i + num_obs_blue);
        }



        this->global_odom = global_odom;
        this->cone_obs_blue.clear();
        this->cone_obs_yellow.clear();
        for (auto b : cone_obs_blue)
        {
            this->cone_obs_blue.push_back(b);
        }
        for (auto y : cone_obs_yellow)
        {
            this->cone_obs_yellow.push_back(y);
        }

        RCLCPP_INFO(logger, "copied observations");
        auto end_vectorization = high_resolution_clock::now();
        auto dur_v = duration_cast<microseconds>(end_vectorization - start);
        RCLCPP_INFO(logger, "vectorization time: %d", dur_v.count());
        // vector<Point2> *heap_obs_blue = new vector<Point2>;
        // vector<Point2> *heap_obs_yellow = new vector<Point2>;

        // for (auto b : cone_obs_blue)
        // {
        //     heap_obs_blue->push_back(b);
        // }

        // for (auto y : cone_obs_yellow)
        // {
        //     heap_obs_yellow->push_back(y);
        // }

        

        /* Firstly, data associate on the last HEURISTIC_N landmark estimates */
        int blue_multiple_size = HEURISTIC_N+1;
        int yellow_multiple_size = HEURISTIC_N+1;

        int blue_lo = blue_n_landmarks - HEURISTIC_N;
        int yellow_lo = yellow_n_landmarks - HEURISTIC_N;
        if (!heuristic_run || HEURISTIC_N >= blue_n_landmarks || HEURISTIC_N >= yellow_n_landmarks)
        {
            /* cannot use HEURISTIC_N => Look at all previous cone estimates */
            blue_multiple_size = blue_n_landmarks + 1;
            yellow_multiple_size = yellow_n_landmarks + 1;

            blue_lo = 0;
            yellow_lo = 0;

        }

        this->blue_cone_est.clear();
        this->yellow_cone_est.clear();
        RCLCPP_INFO(logger, "estimate vectors cleared");


        /* Getting the most recent HEURISTIC_N number previous cones */
        for (int i = blue_lo; i < blue_n_landmarks; i++)
        {
            int id = blue_cone_IDs.at(i);
            this->blue_cone_est.push_back(isam2.calculateEstimate(L(id)).cast<Pose2>());
        }

        for (int i = yellow_lo; i < yellow_n_landmarks; i++)
        {
            int id = yellow_cone_IDs.at(i);
            this->yellow_cone_est.push_back(isam2.calculateEstimate(L(id)).cast<Pose2>());
        }


        RCLCPP_INFO(logger, "finished vectorization");
        RCLCPP_INFO(logger, "Previously: blue_n_landmarks: %d | yellow_n_landmarks: %d",
                                    blue_n_landmarks, yellow_n_landmarks);
        /* previous MinID_Args need to be processed first before the new
         * Assoc_Args can be added to the work_queue*/



        /* start new data association cycle */

        /* Retrieve the blue observed cones and the yellow observed cones */

        /* Add Assoc_Args tasks to the work queue */

        if (num_obs_blue == 0 && num_obs_yellow == 0) /* no observations */
        {
            RCLCPP_INFO(logger, "no observed cones");
            prev_DA_done = true;
            //pose_num++;
            step_cv.notify_one();
            return;
        }




        int m_dist_len = (blue_multiple_size * (int)cone_obs_blue.size() +
                            yellow_multiple_size * (int)cone_obs_yellow.size());

        m_dist.resize(m_dist_len);

        int multiple_size = (int)(m_dist_len / NUM_THREADS);
        int remainders = m_dist_len - (NUM_THREADS * multiple_size);
        if (m_dist_len < MIN_M_DIST || (blue_n_landmarks == 0 && yellow_n_landmarks == 0))
        {
            multiple_size = m_dist_len;
            remainders = 0;
        }
        int lo = 0;
        int hi = 0;

        //bool is_first_DA = (blue_n_landmarks == 0 && yellow_n_landmarks == 0);

        work_queue_mutex.lock();
        int num_tasks = 0;
        while (lo < m_dist_len && hi < m_dist_len)
        {
            hi = lo + multiple_size;

            if (remainders > 0)
            {
                hi++;
                remainders--;
            }
            RCLCPP_INFO(logger, "step adding Assoc_Args; lo: %d | hi: %d", lo, hi);
            Assoc_Args *A_task = new 
            Assoc_Args(&(this->cone_obs_blue), &(this->cone_obs_yellow), lo, hi);


            work_queue.push_back(make_tuple(A_task, 'a'));
            num_tasks++;


            lo = hi;
        }
        RCLCPP_INFO(logger, "work_queue size: %d; num_tasks: %d",
                    (int)work_queue.size(), num_tasks);
        work_queue_mutex.unlock();

        auto end_add_assoc = high_resolution_clock::now();
        auto dur_end_add_assoc = duration_cast<microseconds>(end_add_assoc - start);

        RCLCPP_INFO(logger, "added Assoc_Args to work queue; m_dist len = %d | time: %d",
                                (int)m_dist.size(), dur_end_add_assoc.count());
        /*Proof: Assoc_Args are being added every time*/
        //assert('d' == 'z');


        /* Create a boolean to check if mahalanobis calcs are done for current
         * time step before proceeding to mahalanobis calcs for next time step
         *
         */

        std::ofstream ofs;
        std::ofstream out("squirrel.txt");
        std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
        std::cout.rdbuf(out.rdbuf());
        ofs.open("squirrel.txt", std::ofstream::out | std::ofstream::trunc);
        auto estimate = isam2.calculateEstimate();
        estimate.print("Estimate:");
        ofs.close();
        std::cout.rdbuf(coutbuf); //reset to standard output again


        //calculate estimate of robot state
        //robot_est = isam2.calculateEstimate().at(X(pose_num)).cast<gtsam::Pose2>();
        //pose_num++;
        // std::cout << "Robot Estimate:(" << robot_est.x() <<"," << robot_est.y() << ")" << std::endl;
    }








    /**
     * @brief
     *
     * @param measurement: the global position of the observed cone at
     * current time step
     *
     * @param landmark; the isam2 estimate for global position of previously
     * seen cone. we want the covariance of this cone
     */
    double mahalanobisDist(auto logger, Pose2 measurement,Pose2 landmark,Symbol landmark_key){
        //mahalanobis distance with just x,y???
        Eigen::MatrixXd diff(1, 3);
        diff << measurement.x()-landmark.x(),measurement.y()-landmark.y(),1;
        Eigen::MatrixXd marginal_covariance = isam2.marginalCovariance(landmark_key);
        /*RCLCPP_INFO(logger, "\n%f | %f | %f\n %f | %f | %f\n %f | %f | %f\n ",
                    marginal_covariance(0, 0), marginal_covariance(0, 1), marginal_covariance(0, 2),
                    marginal_covariance(1, 0), marginal_covariance(1, 1), marginal_covariance(1, 2),
                    marginal_covariance(2, 0), marginal_covariance(2, 1), marginal_covariance(2, 2));*/

        //shouldn't this be inverse?

        Eigen::MatrixXd test = diff * marginal_covariance;
        //RCLCPP_INFO(logger, "\n%f | %f | %f\n ",
        //        test(0, 0), test(0, 1), test(0, 2));

        Eigen::MatrixXd result = diff*marginal_covariance*diff.transpose();
        //size of eigen matrix is (1,1)
        return result(0);
    }

    /**
     * @brief print_cones will print the positions of the
     * observed cones stored within the vector cone_obs
     *
     * @param cone_obs is a memory address to the vector
     * containing the Point2 positions of the observed cones
     * - you can just pass in the variable, but the type is
     *   std::vector<Point2> &
     */
    void print_cones(auto logger, std::vector<Point2> &cone_obs)
    {
        RCLCPP_INFO(logger, "Printing cones");
        for (int i = 0; i < cone_obs.size(); i++)
        {
            RCLCPP_INFO(logger, "(%f, %f)\n", cone_obs.at(i).x(),
                                            cone_obs.at(i).y());
        }
        RCLCPP_INFO(logger, "done printing\n\n");
    }



    



};
