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
//static const float M_DIST_TH = 0.000151169; // for real data
static const float M_DIST_TH = 25;
// static const float M_DIST_TH = 45; // used to be 45 lmao
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

    /* You need both because you could switch over from computing blue observations
     * to computing yellow computations while in the same thread
     */
    Eigen::MatrixXd blue_global_cone_x;
    Eigen::MatrixXd blue_global_cone_y;

    Eigen::MatrixXd yellow_global_cone_x;
    Eigen::MatrixXd yellow_global_cone_y;

    Eigen::MatrixXd blue_bearing;
    Eigen::MatrixXd yellow_bearing;

    vector<Pose2> *blue_cone_est;
    vector<Pose2> *yellow_cone_est;
    Pose2 global_odom;

    vector<float> *m_dist;
    int lo;
    int hi;


    /* cone_obs must be a pointer be thread_safe */
    Assoc_Args(vector<Point2> *cone_obs_blue, vector<Point2> *cone_obs_yellow,
            Eigen::MatrixXd blue_global_cone_x, Eigen::MatrixXd blue_global_cone_y,
            Eigen::MatrixXd yellow_global_cone_x, Eigen::MatrixXd yellow_global_cone_y,
            Eigen::MatrixXd blue_bearing, Eigen::MatrixXd yellow_bearing,
            vector<Pose2> *blue_cone_est, vector<Pose2> *yellow_cone_est,
            Pose2 global_odom, vector<float> *m_dist, int lo, int hi)
    {
        /* separate because when you data associate and fine a new cone,
         * you want to know the color of the new cone you add */
        this->cone_obs_blue = cone_obs_blue;
        this->cone_obs_yellow = cone_obs_yellow;

        /* you'll need to separate */
        this->blue_global_cone_x = blue_global_cone_x;
        this->blue_global_cone_y = blue_global_cone_y;

        this->yellow_global_cone_x = yellow_global_cone_x;
        this->yellow_global_cone_y = yellow_global_cone_y;

        this->blue_bearing = blue_bearing;
        this->yellow_bearing = yellow_bearing;

        /* cannot separate all_cone_est because access to marginal cov is also like this */
        /* Not necessarily: the index in color_cone_est indicates the index in color_cone_IDs to access*/

        this->blue_cone_est = blue_cone_est;
        this->yellow_cone_est = yellow_cone_est;

        this->global_odom = global_odom;

        this->m_dist = m_dist;
        this->lo = lo;
        this->hi = hi;

    }
};

class MinID_Args
{
    public:
    vector<float> *m_dist;
    int lo;
    int hi;

    bool is_blue_obs;
    bool is_yellow_obs;

    vector<Point2> *color_cone_obs;
    int color_obs_id;
    vector<int> *color_cone_IDs;
    int prev_color_n_landmarks;

    Pose2 global_odom;

    vector<Point2> *blue_unknown_obs; /* used to hold unknown observations from heur. run */
    vector<Point2> *yellow_unknown_obs;

    /* why not Eigen::MatrixXd; can't heap allocate so can't build amongst many diff MinID_Args */
    vector<Pose2> *blue_glob_obs;
    vector<Pose2> *yellow_glob_obs;

    int num_obs;

    Pose2 glob_pos_bearing;


    MinID_Args (bool is_blue_obs, bool is_yellow_obs, vector<Point2> *color_cone_obs,
                int prev_color_n_landmarks, int color_obs_id, vector<int> *color_cone_IDs,
                Pose2 global_odom,
                vector<Point2> *blue_unknown_obs, vector<Point2> *yellow_unknown_obs,
                vector<Pose2> *blue_glob_obs, vector<Pose2> *yellow_glob_obs,
                int num_obs, Pose2 glob_pos_bearing, vector<float> *m_dist, int lo, int hi)
    {
        this->is_blue_obs = is_blue_obs;
        this->is_yellow_obs = is_yellow_obs;

        this->color_cone_obs = color_cone_obs;

        this->prev_color_n_landmarks = prev_color_n_landmarks;
        this->color_obs_id = color_obs_id;
        this->color_cone_IDs = color_cone_IDs;

        this->global_odom = global_odom;

        this->blue_unknown_obs = blue_unknown_obs;
        this->yellow_unknown_obs = yellow_unknown_obs;

        this->blue_glob_obs = blue_glob_obs;
        this->yellow_glob_obs = yellow_glob_obs;

        this->num_obs = num_obs;

        this->glob_pos_bearing = glob_pos_bearing;

        this->m_dist = m_dist;
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


    gtsam::Symbol X(int robot_pose_id) {
        return Symbol('x', robot_pose_id);
    }

    gtsam::Symbol L(int cone_pose_id) {
        return Symbol('l', cone_pose_id);
    }

public:

    int n_landmarks;
    int blue_n_landmarks;
    int yellow_n_landmarks;

    bool heuristic_run;
    atomic<bool> prev_DA_done;

    //gtsam::Pose2 robot_est;
    std::vector<gtsam::Pose2> landmark_est;
    std::vector<Point2> orange_cones;

    /* how the landmark estimates are organized */
    vector<int> blue_cone_IDs;
    vector<int> yellow_cone_IDs;


    slamISAM(rclcpp::Logger logger) {
        parameters = ISAM2Params(ISAM2DoglegParams(),0.1,10,true);
        parameters.setFactorization("QR");

        isam2 = gtsam::ISAM2(parameters);
        graph = gtsam::NonlinearFactorGraph();
        values = gtsam::Values();
        pose_num = 0;
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
        Vector LandmarkNoiseModel(3);
        //used to be 0.01 for real data
        //LandmarkNoiseModel(0) = 0.01;
        //LandmarkNoiseModel(1) = 0.01;
        //LandmarkNoiseModel(2) = 0.01;

        // No noise kinda good??
        LandmarkNoiseModel(0) = 0.0;
        LandmarkNoiseModel(1) = 0.0;
        LandmarkNoiseModel(2) = 0.0;
        //print_cones(logger, cone_obs);
        static auto landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);



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
                                    A_task->blue_bearing(i, 0)), landmark_model));

                values.insert(L(n_landmarks), Pose2(A_task->blue_global_cone_x(i),
                                                    A_task->blue_global_cone_y(i),
                                                        0));
                if (n_landmarks == 0)
                {
                    RCLCPP_INFO(logger, "first landmark added");
                }

	    	isam2.update(graph, values);
            	isam2.update();
            	values.clear();
            	graph.resize(0);

                blue_cone_IDs.push_back(n_landmarks);
                blue_n_landmarks++;
                n_landmarks++;

            }

            for (int i = 0; i < num_obs_yellow; i++)
            {
                graph.add(BetweenFactor<Pose2>(X(pose_num), L(n_landmarks),
                            Pose2(A_task->cone_obs_yellow->at(i).x(),
                                    A_task->cone_obs_yellow->at(i).y(),
                                    A_task->yellow_bearing(i, 0)), landmark_model));

                 values.insert(L(n_landmarks), Pose2(A_task->yellow_global_cone_x(i),
                                                    A_task->yellow_global_cone_y(i),
                                                        0));

                if (n_landmarks == 0)
                {
                    RCLCPP_INFO(logger, "first landmark added");
                }

		isam2.update(graph, values);
            	isam2.update();
            	values.clear();
            	graph.resize(0);

                yellow_cone_IDs.push_back(n_landmarks);
                yellow_n_landmarks++;
                n_landmarks++;
            }


            
            prev_DA_done = true;
            pose_num++;
            step_cv.notify_one();

            isam2_mutex.unlock();

            RCLCPP_INFO(logger, "completed first DA cycle");
            return;


        }

        /* For each blue observed cone, calculate Mahalanobis distance wrt blue_n_landmarks
         * The +1 represents the M_DIST_TH for that observation
         *
         * Likewise for each yellow observed cone
         */
        int blue_multiple_size = blue_n_landmarks + 1;
        int yellow_multiple_size = yellow_n_landmarks + 1;
        bool using_heuristic_n = false;
        /* Use when there's many previous estimates */
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
            color_cone_est = A_task->blue_cone_est;
            obs_id = (int)(A_task->lo / blue_multiple_size);
            ith_color_cone = A_task->lo % blue_multiple_size;
        }
        else if (!cur_obs_is_blue)
        {
            /* for indexing into cone_obs_yelow */
            color_cone_est = A_task->yellow_cone_est;
            int offset_start = (int)(A_task->lo - first_yellow_idx);
            obs_id = (int)(offset_start / yellow_multiple_size);
            ith_color_cone = offset_start % yellow_multiple_size;
        }


        while (i < A_task->hi)
        {
            int last_idx_for_cur_obs;
            vector<int> *cone_IDs;
            Eigen::MatrixXd *global_cone_x;
            Eigen::MatrixXd *global_cone_y;

            /* last_est_for_cur_obs: the last index in m_dist to calculate up to
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

            if (cur_obs_is_blue) /* blue observation */
            {
                last_idx_for_cur_obs = ((obs_id + 1) *
                                            blue_multiple_size - 1);
                cone_IDs = &blue_cone_IDs;
                global_cone_x = &A_task->blue_global_cone_x;
                global_cone_y = &A_task->blue_global_cone_y;

            }
            else if (!cur_obs_is_blue) /* yellow observations */
            {
                last_idx_for_cur_obs = (first_yellow_idx +
                                        ((obs_id + 1) *
                                         yellow_multiple_size) - 1);
                cone_IDs = &yellow_cone_IDs;
                global_cone_x = &A_task->yellow_global_cone_x;
                global_cone_y = &A_task->yellow_global_cone_y;
            }

            /* calculate mahalanobis distance for all previous cones wrt to
             * current obs cone */
            /*RCLCPP_INFO(logger,"\nlo: %d, hi: %d \n obs_id: %d | num_obs_blue: %d | num_obs_yellow: %d \n"
                                "first_yellow_idx: %d | blue_n_landmarks: %d | yellow_n_landmarks: %d \n"
                                "last_est_for_cur_obs: %d",
                                A_task->lo, A_task->hi, obs_id, num_obs_blue, num_obs_yellow, first_yellow_idx,
                                blue_n_landmarks, yellow_n_landmarks, last_est_for_cur_obs); */
            RCLCPP_INFO(logger, "lo: %d | hi: %d | blue_n_landmarks: %d | yellow_n_landmarks: %d"
                                "first_yellow_idx: %d", A_task->lo, A_task->hi, blue_n_landmarks,
                                yellow_n_landmarks, first_yellow_idx);


            for (; i < last_idx_for_cur_obs && i < A_task->hi; i++)
            {

                Eigen::MatrixXd diff(1, 3);

                //assert(cone_IDs->at(ith_color_cone) < (int)A_task->all_cone_est->size());
                if (cur_obs_is_blue)
                {
                    RCLCPP_INFO(logger, "Blue obs id: %d | blue_cone_IDs size: %d | ith_color_cone: %d",
                                        obs_id, blue_cone_IDs.size(), ith_color_cone);
                }
                else
                {
                    RCLCPP_INFO(logger, "Yellow obs id: %d | yellow_cone_IDs size: %d | ith_color_cone: %d",
                                        obs_id, yellow_cone_IDs.size(), ith_color_cone);
                }


                Pose2 prev_est = color_cone_est->at(ith_color_cone);
                diff << (*global_cone_x)(obs_id, 0) - prev_est.x(),
                        (*global_cone_y)(obs_id, 0) - prev_est.y(),
                        1;
                int id = cone_IDs->at(ith_color_cone);
                if (using_heuristic_n)
                {
		    if (cur_obs_is_blue)
		    {
		        id = cone_IDs->at(blue_n_landmarks - HEURISTIC_N + ith_color_cone);
		    }
		    else 
		    {
			id = cone_IDs->at(yellow_n_landmarks - HEURISTIC_N + ith_color_cone);
		    }
                }
                A_task->m_dist->at(i) = (diff * isam2.marginalCovariance(L(id))
                                              * diff.transpose())(0, 0);
                ith_color_cone++;
                RCLCPP_INFO(logger, "calc id %d of m_dist", i);

            }


            if (i == A_task->hi)
            {
                break;
            }

            A_task->m_dist->at(last_idx_for_cur_obs) = M_DIST_TH;
            i++; /* skip 1 element (reserved for M_DIST_TH */
            obs_id++;

            if (obs_id == num_obs_blue && cur_obs_is_blue)
            {
                obs_id = 0;
                cur_obs_is_blue = false;
                color_cone_est = A_task->yellow_cone_est;
            }
            ith_color_cone = 0;
        }

        assoc_counter = assoc_counter + 1;







        /* Wait for all other Assoc_Args
         * to finish
         *
         * Then start MinID_Args tasks
         *
         */

        if ((assoc_counter == NUM_THREADS) ||
                ((A_task->m_dist->size() < MIN_M_DIST) && (assoc_counter == 1)))
        {
            RCLCPP_INFO(logger, "finished all Assoc_Args");

            /*waiting until all Assoc_Args tasks have finished*/
            int num_obs = (num_obs_blue + num_obs_yellow);

            unique_lock<mutex> assoc_lk(assoc_wait_mutex);
            while (!((assoc_counter == NUM_THREADS) ||
                ((A_task->m_dist->size() < MIN_M_DIST) && (assoc_counter == 1))))
            {
                RCLCPP_INFO(logger, "waiting for Assoc_Args");
                cv.wait(assoc_lk);
            }

            assoc_counter = 0;

            /* add MinIDs_Args to the work queue */
            int lo = 0;
            vector<Point2> *blue_unknown_obs = NULL;
            vector<Point2> *yellow_unknown_obs = NULL;
            vector<Pose2> *blue_glob_obs = NULL;
            vector<Pose2> *yellow_glob_obs = NULL;

            work_queue_mutex.lock();
            RCLCPP_INFO(logger, "num_obs_blue: %d | num_obs_yellow: %d",
                                num_obs_blue, num_obs_yellow);
            for (int i = 0; i < num_obs_blue; i++)
            {
                int hi = lo + blue_n_landmarks + 1;
                if (heuristic_run)
                {
                    hi = lo + HEURISTIC_N + 1;
                    if (blue_n_landmarks < HEURISTIC_N)
                    {
                        hi  = lo + blue_n_landmarks+1;
                    }

                    blue_unknown_obs = new vector<Point2>;
                    yellow_unknown_obs = new vector<Point2>;
                    blue_glob_obs = new vector<Pose2>;
                    yellow_glob_obs = new vector<Pose2>;
                }
                Pose2 glob_pos_bearing = Pose2(A_task->blue_global_cone_x(i, 0),
                                            A_task->blue_global_cone_y(i, 0),
                                            A_task->blue_bearing(i, 0));


                MinID_Args *M_task = new MinID_Args(true, false,
                                                    A_task->cone_obs_blue,
                                                    (int)A_task->blue_cone_est->size(),
                                                    i, &blue_cone_IDs,
                                                    A_task->global_odom,
                                        blue_unknown_obs, yellow_unknown_obs,
                                        blue_glob_obs, yellow_glob_obs,
                                                    num_obs, glob_pos_bearing,
                                                    A_task->m_dist, lo, hi);
                RCLCPP_INFO(logger, "blue id: %d | minID: [%d, %d)", i, lo, hi);
                lo = hi;
                work_queue.push_back(make_tuple(M_task, 'm'));
            }

            for (int i = 0; i < num_obs_yellow; i++)
            {
                int hi = lo + yellow_n_landmarks + 1;
                if (heuristic_run)
                {
                    hi = lo + HEURISTIC_N + 1;
                    if (yellow_n_landmarks < HEURISTIC_N)
                    {
                        hi  = lo + yellow_n_landmarks+1;
                    }
                }

                Pose2 glob_pos_bearing = Pose2(A_task->yellow_global_cone_x(i, 0),
                                            A_task->yellow_global_cone_y(i, 0),
                                            A_task->yellow_bearing(i, 0));

                MinID_Args *M_task = new MinID_Args(false, true,
                                                    A_task->cone_obs_yellow,
                                                    (int)A_task->yellow_cone_est->size(),
                                                    i, &yellow_cone_IDs,
                                                    A_task->global_odom,
                                       blue_unknown_obs, yellow_unknown_obs,
                                       blue_glob_obs, yellow_glob_obs,
                                                    num_obs, glob_pos_bearing,
                                                    A_task->m_dist, lo, hi);
                RCLCPP_INFO(logger, "yellow id: %d | minID: [%d, %d)", i, lo, hi);
                lo = hi;
                work_queue.push_back(make_tuple(M_task, 'm'));
            }

            work_queue_mutex.unlock();
            RCLCPP_INFO(logger, "Last Assoc_Arg added MinID_Args to queue: m_dist_len: %d",
                                    (int)A_task->m_dist->size());

        }

    }

    void find_minIDs(rclcpp::Logger logger, MinID_Args *M_task)
    {
        Vector NoiseModel(3);
        NoiseModel(0) = 0;
        NoiseModel(1) = 0;
        NoiseModel(2) = 0;

        Vector LandmarkNoiseModel(3);
        //used to be 0.01 for real data
        //LandmarkNoiseModel(0) = 0.01;
        //LandmarkNoiseModel(1) = 0.01;
        //LandmarkNoiseModel(2) = 0.01;

        // No noise kinda good??
        LandmarkNoiseModel(0) = 0.0;
        LandmarkNoiseModel(1) = 0.0;
        LandmarkNoiseModel(2) = 0.0;
        //print_cones(logger, cone_obs);
        static auto landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);

        /* identify the min ID */
        /*
            int start_offset = i * (n_landmarks + 1);
            vector<float>::iterator start_iter = m_dist->begin() + start_offset;
            min_ids->at(i) = std::distance(start_iter,
                                        std::min_element(start_iter,
                                            start_iter + (n_landmarks+1)));
        */



        int num_obs = M_task->num_obs;
        vector<float>::iterator start_iter = M_task->m_dist->begin() + M_task->lo;
        vector<float>::iterator end_iter = M_task->m_dist->begin() + M_task->hi;

        int minID = std::distance(start_iter,
                                std::min_element(start_iter, end_iter));

        if (M_task->is_blue_obs)
        {
            RCLCPP_INFO(logger, "minID=%d for blue obs: %d", minID, M_task->color_obs_id);
        }
        else
        {
            RCLCPP_INFO(logger, "minID=%d for yellow obs: %d", minID, M_task->color_obs_id);
        }
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
                    M_task->blue_unknown_obs->push_back(
                        M_task->color_cone_obs->at(M_task->color_obs_id));
                    M_task->blue_glob_obs->push_back(M_task->glob_pos_bearing);
                }
                else if (M_task->is_yellow_obs && !(M_task->is_blue_obs)) /*is yellow*/
                {
                    M_task->yellow_unknown_obs->push_back(
                        M_task->color_cone_obs->at(M_task->color_obs_id));
                    M_task->yellow_glob_obs->push_back(M_task->glob_pos_bearing);
                }
            }
            else /* not heuristic_run => add to isam2 */
            {
                int obs_id = M_task->color_obs_id;
                graph.add(BetweenFactor<Pose2>(X(pose_num), L(n_landmarks),
                                        Pose2(M_task->color_cone_obs->at(obs_id).x(),
                                            M_task->color_cone_obs->at(obs_id).y(),
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
            if (M_task->is_blue_obs)
            {
                RCLCPP_INFO(logger, "minID: %d old cone; blue_cone_IDs size: %d vs %d",
                                minID, (int)blue_cone_IDs.size(),
                                (int)M_task->color_cone_IDs->size());
            }
            else
            {
                RCLCPP_INFO(logger, "minID: %d old cone; yellow_cone_ids size: %d",
                                minID, (int)yellow_cone_IDs.size());
            }


            int obs_id = M_task->color_obs_id;
            int id = -1;
            if (!heuristic_run || HEURISTIC_N >= blue_n_landmarks || HEURISTIC_N >= yellow_n_landmarks)
            {
                /* not using HEURISTIC_N */
                id = M_task->color_cone_IDs->at(minID);
            }
            else
            {
                /* using HEURISTIC_N */
                id = M_task->color_cone_IDs->at(M_task->prev_color_n_landmarks - HEURISTIC_N + minID);
            }


            graph.add(BetweenFactor<Pose2>(X(pose_num), L(id),
                                        Pose2(M_task->color_cone_obs->at(obs_id).x(),
                                            M_task->color_cone_obs->at(obs_id).y(),
                                            M_task->glob_pos_bearing.theta()),
                                            landmark_model));

        }


        ///////////////////////////////////////////////////////////////////////
        RCLCPP_INFO(logger, "min id %d", M_task->color_obs_id);
        isam2.update(graph, values);
        graph.resize(0);
        values.clear();

        minID_counter = minID_counter + 1;


        /* Is calling cv.wait for a thread really necessary?
         * No: if minID_counter == num_obs and if only 1 thread can be updating
         * isam2 at a time, then that means all MinID_Args are finished
         * when they are processed 1 at a time
         */


        isam2_mutex.unlock();
        /* wait for all MinID_Args to be finished */
        /* Take the first MinID_Args; No way to actually get the last by color
         * You may not always have blue cones; you may not always have yellow cones*/
        if (minID_counter == num_obs)
        {

            minID_counter = 0;
            RCLCPP_INFO(logger, "finished processing last minID | is heuristic_run: %d", heuristic_run);

            int blue_num_u_obs = 0;
            int yellow_num_u_obs = 0;
            int num_obs = 0;

            if (heuristic_run)
            {
                blue_num_u_obs = (int)M_task->blue_unknown_obs->size();
                yellow_num_u_obs = (int)M_task->yellow_unknown_obs->size();
                num_obs = blue_num_u_obs + yellow_num_u_obs;
            }



            if (heuristic_run && num_obs > 0)
            {
                heuristic_run = false;

                /* create the new global_cone x and y and bearing */
                Eigen::MatrixXd blue_bearing = Eigen::MatrixXd(blue_num_u_obs, 1);
                Eigen::MatrixXd yellow_bearing = Eigen::MatrixXd(yellow_num_u_obs, 1);

                Eigen::MatrixXd blue_global_cone_x = Eigen::MatrixXd(blue_num_u_obs, 1);
                Eigen::MatrixXd blue_global_cone_y = Eigen::MatrixXd(blue_num_u_obs, 1);

                Eigen::MatrixXd yellow_global_cone_x = Eigen::MatrixXd(yellow_num_u_obs, 1);
                Eigen::MatrixXd yellow_global_cone_y = Eigen::MatrixXd(yellow_num_u_obs, 1);

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

                vector<Pose2> *blue_cone_est = new vector<Pose2>;
                vector<Pose2> *yellow_cone_est = new vector<Pose2>;

                for (int i = 0; i < blue_n_landmarks; i++)
                {
                    int id = blue_cone_IDs.at(i);
                    blue_cone_est->push_back(isam2.calculateEstimate(L(id)).cast<Pose2>());
                }

                for (int i = 0; i < yellow_n_landmarks; i++)
                {
                    int id = yellow_cone_IDs.at(i);
                    yellow_cone_est->push_back(isam2.calculateEstimate(L(id)).cast<Pose2>());
                }

                /* Add Assoc_Args for unknown_obs */
                int m_dist_len = ((blue_n_landmarks + 1) * blue_num_u_obs +
                            (yellow_n_landmarks + 1) * yellow_num_u_obs);
                delete M_task->m_dist;
                delete M_task->blue_glob_obs;
                delete M_task->yellow_glob_obs;
                vector<float> *m_dist = new vector(m_dist_len, (float)0.0);
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

                    Assoc_Args *A_task = new Assoc_Args(M_task->blue_unknown_obs, M_task->yellow_unknown_obs,
                                                    blue_global_cone_x, blue_global_cone_y,
                                                yellow_global_cone_x, yellow_global_cone_y,
                                                blue_bearing, yellow_bearing,
                                                blue_cone_est, yellow_cone_est,
                                                M_task->global_odom, m_dist, lo, hi);
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
                pose_num++;
                step_cv.notify_one();

                RCLCPP_INFO(logger, "finished prev DA");
            }



        }


        /* if heuristic_run, after filling up the unknown_obs, then you have to
         * do data association again
         */
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
                RCLCPP_INFO(logger, "work queue size = %d; t_id: %d",
                                    (int)work_queue.size(), t_id);
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
                vector<Point2> &orange_ref_cones, gtsam::Point2 velocity,
                long time_ns, bool loopClosure)
    {
        //assert('s' == 'z'); Proved that nothing wrong with ros topics
        unique_lock<mutex> step_lk(step_wait_mutex);
        while (prev_DA_done == false)
        {
            RCLCPP_INFO(logger, "waiting at step for prev DA");
            step_cv.wait(step_lk);
        }

        prev_DA_done = false;

        Vector NoiseModel(3);
        NoiseModel(0) = 0;
        NoiseModel(1) = 0;
        NoiseModel(2) = 0;

        Vector LandmarkNoiseModel(3);
        //used to be 0.01 for real data
        //LandmarkNoiseModel(0) = 0.01;
        //LandmarkNoiseModel(1) = 0.01;
        //LandmarkNoiseModel(2) = 0.01;

        // No noise kinda good??
        LandmarkNoiseModel(0) = 0.0;
        LandmarkNoiseModel(1) = 0.0;
        LandmarkNoiseModel(2) = 0.0;
        //print_cones(logger, cone_obs);
        static auto landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);

        if (pose_num==0) {//if this is the first pose, add your inital pose to the factor graph
            //std::cout << "First pose\n" << std::endl;
            static noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Diagonal::Sigmas(NoiseModel);
            gtsam::PriorFactor<Pose2> prior_factor = gtsam::PriorFactor<Pose2>(X(0), global_odom, prior_model);
            //add prior
            graph.add(prior_factor);
            values.insert(X(0), global_odom);


            //ASSUMES THAT YOU SEE ORANGE CONES ON YOUR FIRST MEASUREMENT OF LANDMARKS
            //Add orange cone left and right
            //hopefully it's only 2 cones
            orange_cones = orange_ref_cones;
        }
        else {
            //std::cout << "New Pose\n" << std::endl;
            static noiseModel::Diagonal::shared_ptr odom_model = noiseModel::Diagonal::Sigmas(NoiseModel);
            Pose2 prev_pos = isam2.calculateEstimate(X(pose_num - 1)).cast<Pose2>();
            //create a factor between current and previous robot pose
            //add odometry estimates
            //Motion model


            //TODO: change back to motion model with velocity
            double time_s = time_ns/SEC_TO_NANOSEC;

            Pose2 Odometry =  Pose2(velocity.x()*time_s, velocity.y()*time_s, global_odom.theta() - prev_pos.theta());
            /*real data motion model?
            Pose2 Odometry =  Pose2(global_odom.x() - prev_pos.x(),global_odom.y() - prev_pos.y(),
                                    global_odom.theta() - prev_pos.theta());
                                    */

            static noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Diagonal::Sigmas(NoiseModel);
            gtsam::PriorFactor<Pose2> prior_factor = gtsam::PriorFactor<Pose2>(X(0), global_odom, prior_model);
            /* add prior */
            graph.add(prior_factor);

            gtsam::BetweenFactor<Pose2> odom_factor = gtsam::BetweenFactor<Pose2>(X(pose_num - 1), X(pose_num),
                                                                                    Odometry, odom_model);
            graph.add(odom_factor);
            values.insert(X(pose_num), global_odom);
        }

        RCLCPP_INFO(logger, "beginning loop closure");
        if(loopClosure){
            // std::cout<<"loop closure constraint added"<<std::endl;
            static noiseModel::Diagonal::shared_ptr loop_closure_model = noiseModel::Diagonal::Sigmas(NoiseModel);
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
        graph.resize(0);
        values.clear();
        /*TODO: potential issue is data associating the next time step before
         * updating the current iSAM2 model with info of the current time step.
         * - This will affect data association because the new cones from time
         *   step n are not added yet before n+1 is processed
         */

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

        Eigen::MatrixXd blue_global_cone_x = global_cone_x.block(0, 0,
                                                    num_obs_blue, 1);
        Eigen::MatrixXd blue_global_cone_y = global_cone_y.block(0, 0,
                                                    num_obs_blue, 1);

        Eigen::MatrixXd yellow_global_cone_x = global_cone_x.block(num_obs_blue, 0,
                                                    num_obs_yellow, 1);

        Eigen::MatrixXd yellow_global_cone_y = global_cone_y.block(num_obs_blue, 0,
                                                    num_obs_yellow, 1);

        Eigen::MatrixXd blue_bearing = Eigen::MatrixXd(num_obs_blue, 1);
        Eigen::MatrixXd yellow_bearing = Eigen::MatrixXd(num_obs_yellow, 1);

        for (int i = 0; i < num_obs_blue; i++)
        {
            blue_bearing(i) = bearing(i);
        }

        for (int i = 0; i < num_obs_yellow; i++)
        {
            yellow_bearing(i) = bearing(i + num_obs_blue);
        }





        vector<Point2> *heap_obs_blue = new vector<Point2>;
        vector<Point2> *heap_obs_yellow = new vector<Point2>;

        for (auto b : cone_obs_blue)
        {
            heap_obs_blue->push_back(b);
        }

        for (auto y : cone_obs_yellow)
        {
            heap_obs_yellow->push_back(y);
        }

        RCLCPP_INFO(logger, "finished vectorization");

        /* Firstly, data associate on the last HEURISTIC_N landmark estimates */
        int blue_multiple_size = HEURISTIC_N+1;
        int yellow_multiple_size = HEURISTIC_N+1;

        vector<Pose2> *blue_cone_est = new vector<Pose2>;
        vector<Pose2> *yellow_cone_est = new vector<Pose2>;
        int blue_lo = blue_n_landmarks - HEURISTIC_N;
        int yellow_lo = yellow_n_landmarks - HEURISTIC_N;
        if (HEURISTIC_N >= blue_n_landmarks || HEURISTIC_N >= yellow_n_landmarks)
        {
            /* cannot use HEURISTIC_N => Look at all previous cone estimates */
            blue_multiple_size = blue_n_landmarks + 1;
            yellow_multiple_size = yellow_n_landmarks + 1;

            blue_lo = 0;
            yellow_lo = 0;

        }

        /* Getting the most recent HEURISTIC_N number previous cones */
        for (int i = blue_lo; i < blue_n_landmarks; i++)
        {
            int id = blue_cone_IDs.at(i);
            blue_cone_est->push_back(isam2.calculateEstimate(L(id)).cast<Pose2>());
        }

        for (int i = yellow_lo; i < yellow_n_landmarks; i++)
        {
            int id = yellow_cone_IDs.at(i);
            yellow_cone_est->push_back(isam2.calculateEstimate(L(id)).cast<Pose2>());
        }



        /* previous MinID_Args need to be processed first before the new
         * Assoc_Args can be added to the work_queue*/



        /* start new data association cycle */

        /* Retrieve the blue observed cones and the yellow observed cones */

        /* Add Assoc_Args tasks to the work queue */

        if (num_obs_blue == 0 && num_obs_yellow == 0) /* no observations */
        {
            RCLCPP_INFO(logger, "no observed cones");
            prev_DA_done = true;
            pose_num++;
            step_cv.notify_one();
            return;
        }




        int m_dist_len = (blue_multiple_size * (int)cone_obs_blue.size() +
                            yellow_multiple_size * (int)cone_obs_yellow.size());

        vector<float> *m_dist = new vector(m_dist_len, (float)0);

        int multiple_size = (int)(m_dist_len / NUM_THREADS);
        int remainders = m_dist_len - (NUM_THREADS * multiple_size);
        if (m_dist_len < MIN_M_DIST)
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
            Assoc_Args *A_task = new Assoc_Args(heap_obs_blue, heap_obs_yellow,
                    blue_global_cone_x, blue_global_cone_y,
                    yellow_global_cone_x, yellow_global_cone_y,
                    blue_bearing, yellow_bearing,
                    blue_cone_est, yellow_cone_est,
                    global_odom, m_dist, lo, hi);


            work_queue.push_back(make_tuple(A_task, 'a'));
            num_tasks++;


            lo = hi;
        }
        RCLCPP_INFO(logger, "work_queue size: %d; num_tasks: %d",
                    (int)work_queue.size(), num_tasks);
        work_queue_mutex.unlock();

        RCLCPP_INFO(logger, "added Assoc_Args to work queue; m_dist len = %d",
                                (int)m_dist->size());
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



    /**
     * @brief t_associate is a function called by threads that will
     * perform data association on the observed cones stored as Point2
     * inside vector cone_obs.
     *
     * the thread will operate on cones indicated by the indices in the
     * range of [lo, hi)
     */
    void t_associate(vector<Point2> *cone_obs, Eigen::MatrixXd* global_cone_x,
	    Eigen::MatrixXd* global_cone_y, vector<Pose2> *all_cone_est,
	    Pose2 global_odom, vector<float> *m_dist, int lo, int hi)
    {
        int i = lo;
        int landmark_idx = lo % (n_landmarks + 1);
        int obs_id = (int)(lo / (n_landmarks + 1));
        //RCLCPP_INFO(logger, "lo: %d | hi: %d", lo, hi);


        while (i < hi && obs_id < cone_obs->size())
        {

            /**
             * calculate for how many previous cone estimates to calculate
             * mahalanobis distance for, with respect to the current observation.
             *
             */

            /**
             * n_landmarks + 1: add 1 for M_DIST_TH
             *
             * -1: subtract 1 for last landmark index and to exclude current M_DIST_TH
             *
             */

            int last_est_for_cur_obs = (obs_id + 1) * (n_landmarks + 1) - 1;
            //RCLCPP_INFO(logger, "Obs_id: %d",obs_id);
            for (; i < last_est_for_cur_obs && i < hi; i++)
            {

                Eigen::MatrixXd diff(1, 3);
                //RCLCPP_INFO(logger, "Last est idx: %d | Cur lm_idx: %d | i: %d | hi: %d", last_est_for_cur_obs, landmark_idx, i, hi);
                Pose2 prev_est = all_cone_est->at(landmark_idx);
                //RCLCPP_INFO(logger, "accessed est. pose | i: %d | x: %f, y%f ", i, prev_est.x(), prev_est.y());
                diff << (*global_cone_x)(obs_id) - prev_est.x(),
                        (*global_cone_y)(obs_id) - prev_est.y(),
                        1;


                //RCLCPP_INFO(logger, "calculated estimate and diff");
                m_dist->at(i) = (diff * isam2.marginalCovariance(L(landmark_idx)) * diff.transpose())(0, 0);
                //RCLCPP_INFO(logger, "accessed marg.cov. | i: %d", i);
                //RCLCPP_INFO(logger, "calculated mahal distance");

                landmark_idx++;
            }

            if (i == hi)
            {
                return;
            }
            m_dist->at(last_est_for_cur_obs) = M_DIST_TH;
            i++; /* skip 1 element (reserved for M_DIST_TH */
            obs_id++;
            landmark_idx = 0;
        }

    }


    void t_find_min_ids(vector<float> *m_dist, vector<int> *min_ids,
                                    int cone_obs_lo, int cone_obs_hi)
    {
        for (int i = cone_obs_lo; i < cone_obs_hi; i++)
        {
            int start_offset = i * (n_landmarks + 1);
            vector<float>::iterator start_iter = m_dist->begin() + start_offset;
            min_ids->at(i) = std::distance(start_iter,
                                        std::min_element(start_iter,
                                            start_iter + (n_landmarks+1)));
        }
    }



};
