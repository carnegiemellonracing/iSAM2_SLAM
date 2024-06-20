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
#include <chrono>
#include <iostream>
#include <fstream>
using namespace std;
using namespace gtsam;
using namespace std::chrono;

//static const float M_DIST_TH = 0.000151169; // for real data
static const float M_DIST_TH = 25;
// static const float M_DIST_TH = 45; // used to be 45 lmao
static const long SEC_TO_NANOSEC = 1000000000;
//static mutex global_obs_cones_mutex;
static mutex isam2_mutex;

// static const float DT = 0.1;
// static const float SIM_TIME = 50.0;
// static const int LM_SIZE = 2;
// static const int STATE_SIZE = 3;
// static const int N_STEP = 100;

struct Landmark {
    int lm_id;
    gtsam::Pose2 lm_pos;
};

class Compare {
public:
    bool operator()(Landmark lm1, Landmark lm2) const {
        return (lm1.lm_pos.x() > lm2.lm_pos.x() && lm1.lm_pos.y() > lm2.lm_pos.y());
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

    int n_landmarks; gtsam::Pose2 robot_est;
    std::vector<gtsam::Pose2> landmark_est;
    std::vector<Point2> orange_cones;

    slamISAM() {
        parameters = ISAM2Params(ISAM2DoglegParams(),0.1,10,true);
        parameters.setFactorization("QR");

        isam2 = gtsam::ISAM2(parameters);
        graph = gtsam::NonlinearFactorGraph();
        values = gtsam::Values();
        pose_num = 0;
        n_landmarks = 0;
        robot_est = gtsam::Pose2(0, 0, 0);
        landmark_est = std::vector<gtsam::Pose2>();

        orange_cones = std::vector<Point2>();
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





    //returns associated landmark id or n_landmarks if there is no associated id
    //returns zero on the first
    int associate(auto logger, Pose2 measurement) {
        // Vector that will store mahalanobis distances
        std::vector<double> min_dist;

        // Previous one
        //vectorize this
        //Eigen::VectorXd v_m_dist = v_associate(logger, measurement);

        RCLCPP_INFO(logger, "printing m_dist");
        for (int i = 0; i < n_landmarks; i++) {
            gtsam::Pose2 landmark = isam2.calculateEstimate(L(i)).cast<Pose2>();

            // Adding mahalanobis distance to minimum distance vector

            //TODO:make this into a matrix and do a single operation on this
            /*RCLCPP_INFO(logger, "actual diff %d: dx: %f \t dy: %f \t d0: %f", i,
                                                                     measurement.x() - landmark.x(),
                                                                     measurement.y() - landmark.y(),
                                                                     1.0);*/

            double mahalanobis = mahalanobisDist(logger, measurement,landmark,L(i));
            // RCLCPP_INFO(logger, "L(%d)=%f",i,mahalanobis);

            //this will already be calculated, so there's no reason to push back
            //RCLCPP_INFO(logger, "p: (%f, %f) | \t m_dist: %f", landmark.x(), landmark.y(), mahalanobis);
            min_dist.push_back(mahalanobis); //i
            //assert(v_m_dist(i) == mahalanobis);
        }
        //RCLCPP_INFO(logger, "min_id: %d | \t M_DIST_TH: %f", n_landmarks, M_DIST_TH);

        //RCLCPP_INFO(logger, "m_dist print done");

        //TODO: you can add this beforehand
        min_dist.push_back(M_DIST_TH); // Add M_DIST_TH for new landmark
        // Find the index of the minimum element in 'min_dist'
        //min_id will be equal to num_landmarks if it didn't find anything under M_DIST_TH



        //TODO:find min
        int min_id = std::distance(min_dist.begin(), std::min_element(min_dist.begin(), min_dist.end()));
        //RCLCPP_INFO(logger, "Min_id: %d \t | \t n_lm: %d\n",min_id, n_landmarks);
        // RCLCPP_INFO(logger, "Min dist %f\n",min_dist[min_id]);

        return min_id;
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

	    /* not needed after vectorization 
            double bearing = std::atan2(cone_obs->at(obs_id).y(),
                                        cone_obs->at(obs_id).x());

            double range = norm2(cone_obs->at(obs_id));

            double global_cone_x = (global_odom.x() + range *
                                            cos(bearing + global_odom.theta()));

            double global_cone_y = (global_odom.y() + range *
                                            sin(bearing + global_odom.theta()));

            global_obs_cones->at(obs_id) = Pose2(global_cone_x, global_cone_y, bearing);
	    */
	

            /**
             * calculate for how many previous cone estimates to calculate
             * mahalanobis distance for, with respect to the current observation.
             *
             */

            /**
             * n_landmarks + 1: add 1 for M_DIST_TH
             *
             * -1: subtract 1 for last landmark index and to exclude M_DIST_TH
             *
             */

            int last_est_for_cur_obs = (obs_id + 1) * (n_landmarks + 1) - 1;
            //RCLCPP_INFO(logger, "Obs_id: %d",obs_id);
            for (; i < last_est_for_cur_obs && i < hi; i++)
            {
                //assert(i < hi);

                Eigen::MatrixXd diff(1, 3);
                //RCLCPP_INFO(logger, "Last est idx: %d | Cur lm_idx: %d | i: %d | hi: %d", last_est_for_cur_obs, landmark_idx, i, hi);
                assert(landmark_idx < n_landmarks);
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
            assert(last_est_for_cur_obs < m_dist->size());
            m_dist->at(last_est_for_cur_obs) = M_DIST_TH;
            i++; /* skip 1 element (reserved for M_DIST_TH */
            obs_id++;
            landmark_idx = 0;
        }



        ///////////////////////////////////////////////////////////////////////
        /*
        for (int i = lo; i < hi; i++)
        {
            double bearing = std::atan2(cone_obs.at(i).y(), cone_obs.at(i).x());
            double range = norm2(cone_obs.at(i));
            double global_cone_x = global_odom.x() + range * cos(bearing + global_odom.theta());
            double global_cone_y = global_odom.y() + range * sin(bearing + global_odom.theta());
            Pose2 global_cone(global_cone_x, global_cone_y, 0);

            float m_dist[n_landmarks+1];
            for (int n = 0; n < n_landmarks; n++)
            {
                Pose2 nth_est = isam2.calculateEstimate(L(n)).cast<Pose2>();
                Eigen::MatrixXd diff(1, 3);
                diff << global_cone_x - nth_est.x(), global_cone_y - nth_est.y(), 1;
                m_dist[lo * n_landmarks + n] = (diff * isam2.marginalCovariance(L(n)) * diff.transpose())(0, 0);
            }
            m_dist[n_landmarks] = M_DIST_TH;
            min_id[i] = std::distance(min_dist.begin(), std::min_element(min_dist.begin(), min_dist.end()));

        }
        */
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

    void step(auto logger, gtsam::Pose2 global_odom, std::vector<Point2> &cone_obs,
                std::vector<Point2> &orange_ref_cones, gtsam::Point2 velocity,
                long time_ns, bool loopClosure) {



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
            //add prior
            graph.add(prior_factor);

            gtsam::BetweenFactor<Pose2> odom_factor = gtsam::BetweenFactor<Pose2>(X(pose_num - 1), X(pose_num),Odometry, odom_model);
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
        // std::cout << "global_odom: "  << global_odom << std::endl;

        RCLCPP_INFO(logger, "DATA ASSOCIATION BEGIN: Pose: (%f, %f)", global_odom.x(),
                                                                        global_odom.y());
        // DATA ASSOCIATION BEGIN


        RCLCPP_INFO(logger, "Printing global obs: (n_landmarks: %d; num_obs: %d)",
                                                    n_landmarks, cone_obs.size());

	Eigen::MatrixXd cone_xy(cone_obs.size(), 2);
	Eigen::MatrixXd range(cone_obs.size(), 1);
	Eigen::MatrixXd bearing(cone_obs.size(), 1);

	int num_obs = (int)cone_obs.size();
	for (int i = 0; i < num_obs; i++)
	{
	    cone_xy(i) = cone_obs.at(i).x(), cone_obs.at(i).y();
	}

	for (int i = 0; i < num_obs; i++)
	{
	    range(i) = norm2(cone_obs.at(i));
	}

	for (int i = 0; i < num_obs; i++)
	{
	    bearing(i) = atan2(cone_obs.at(i).y(), cone_obs.at(i).x());
	}

	Eigen::MatrixXd totalBearing = bearing.array()+global_odom.theta();
	Eigen::MatrixXd global_cone_x(cone_obs.size(),1);
	Eigen::MatrixXd global_cone_y(cone_obs.size(),1);

	Eigen::MatrixXd totalBearing_cos(cone_obs.size(),1);
	Eigen::MatrixXd totalBearing_sin(cone_obs.size(),1);
	totalBearing_cos = totalBearing.array().cos();
	totalBearing_sin = totalBearing.array().sin();

	global_cone_x = global_odom.x() + range.array()*totalBearing_cos.array();
        global_cone_y = global_odom.y() + range.array()*totalBearing_sin.array();

	/*checking correctness of vectorization*/
	/*for (int i = 0; i < num_obs; i++)
	{
	    float true_global_x = global_odom.x() + norm2(cone_obs.at(i)) * cos(global_odom.theta() + bearing(i));
	    float true_global_y = global_odom.y() + norm2(cone_obs.at(i)) * sin(global_odom.theta() + bearing(i));
	    RCLCPP_INFO(logger, "\ntrue: %f, %f \nvect: %f, %f", cos(global_odom.theta() + bearing(i)),
									sin(global_odom.theta() + bearing(i)),
									totalBearing_cos(i), 
								    	totalBearing_sin(i));

	    assert(cos(global_odom.theta() + bearing(i)) == totalBearing_cos(i));
	    assert(sin(global_odom.theta() + bearing(i)) == totalBearing_sin(i));
	    RCLCPP_INFO(logger, "true: %f, %f \nvect: %f, %f", true_global_x, global_cone_x(i, 0), true_global_y, global_cone_y(i, 0));
	    assert((true_global_x - global_cone_x(i, 0)) < 0.00001);
	    assert((true_global_y - global_cone_y(i, 0)) < 0.00001);
    	}*/



        auto start = high_resolution_clock::now();
        if (n_landmarks != 0 && cone_obs.size() > 0)
        {
            int num_threads = 12;

            const int m_dist_len = (n_landmarks + 1) * cone_obs.size(); //110
	     
	    if (m_dist_len < 1000)
	    {
		num_threads = 6;
	    }
	    
	    RCLCPP_INFO(logger, "num threads: %d", num_threads);
            thread all_t[num_threads];
            const int multiple_size = m_dist_len / num_threads; //110/3
            vector<float> m_dist(m_dist_len, 0.0);

            /* spawn num_threads that will are responsible for performing data
             * association on subset of the observed cones
             *
             * populate m_dist
             */

            vector<Pose2> all_cone_est = vector(n_landmarks, Pose2(0, 0, 0));
            for (int i = 0; i < n_landmarks; i++)
            {
                all_cone_est.at(i) = isam2.calculateEstimate(L(i)).cast<Pose2>();
            }

	    auto start_t = high_resolution_clock::now();
            for (int i = 0; i < num_threads; i++)
            {
                all_t[i] = thread(&slamISAM::t_associate, this, &cone_obs, 
				    &global_cone_x, &global_cone_y,
                                    &all_cone_est, global_odom, &m_dist, i * multiple_size,
                                    (i+1) * multiple_size);
                //Debugging: not multi thread
                /*t_associate(logger, &cone_obs, &global_obs_cones, global_odom,
                            &m_dist, i * multiple_size, (i+1) * multiple_size);
                            */
	    }


            for (auto &t : all_t)
            {
                t.join();
            }

            //Compute the remainders
            t_associate(&cone_obs, &global_cone_x, &global_cone_y, &all_cone_est, global_odom, &m_dist,
                                    num_threads * multiple_size, m_dist_len);

            RCLCPP_INFO(logger, "finished populating m_dist\n");


            /* find min_dist; do this in n_landmarks + 1 sub-sections of m_dist*/

            vector<int> min_ids = vector(cone_obs.size(), 0);

            int num_threads2 = (int)cone_obs.size();
            int multiple_size2 = 1;
            if ((int)cone_obs.size() > 12)
            {
                num_threads2 = 8;
                multiple_size2 = cone_obs.size() / num_threads2;
            }

            thread all_t2[num_threads2];
            /**
             * Should you have another thread compute the min_id of the remainder cones?
             * - Worst case: cone_obs.size() == 13
             * - all threads calc. min_id for 1 cone, last thread calc. for 5 cones
             *
             * Should you add the remainder cones to the existing threads?
             * - Worst case: cone_obs.size() == 15
             * - all threads calc. min_id for 1 + 1 cone, last thread calc. for 1 cone
             */

            int remainder = (int)cone_obs.size() - num_threads2 * multiple_size2;
            int cone_obs_idx = 0;
            int threads_idx = 0;

            /**
             * Calculate the lower bound index and assign the thread the cones
             * with indices between cone_obs_idx and next_cone_obs_idx
             *
             * There are no remainders because you disperse the remainders
             * amongs the existing threads
             */
	    
	    
            while (cone_obs_idx < cone_obs.size())
            {
                assert(threads_idx < num_threads2);

                int next_cone_obs_idx = cone_obs_idx + multiple_size2;

                if (remainder > 0)
                {
                    next_cone_obs_idx++;
		    remainder--;
                }

                all_t2[threads_idx] = thread(&slamISAM::t_find_min_ids, this, &m_dist,
                                        &min_ids, cone_obs_idx, next_cone_obs_idx);
                cone_obs_idx = next_cone_obs_idx;
                threads_idx++;
            }

            for (auto& t2 : all_t2)
            {
                t2.join();
            }
	    RCLCPP_INFO(logger, "calc'd min_ids");
	    

	    
	    RCLCPP_INFO(logger, "updating graph");
	    /* the highest min_id could only be prev_n_landmarks */
	    /* update isam2 with any new factors and values */
	    int prev_n_landmarks = n_landmarks;
	    RCLCPP_INFO(logger, "reading min_ids");
            for (int i = 0; i < (int)cone_obs.size(); i++)
            {
		int l_idx = min_ids.at(i);
		
		if (min_ids.at(i) == prev_n_landmarks)
		{
		    l_idx = n_landmarks;
		}
	

		/**
		 * Why wasn't everything blowing up before fixing L(l_idx)
		 * Probably because when you only add 1 new cone at each iteration,
		 * you don't have enough for things to just blow up
		 */
                graph.add(BetweenFactor<Pose2>(X(pose_num), L(l_idx),
                            Pose2(cone_obs.at(i).x(), cone_obs.at(i).y(),
                                bearing(i, 0)),
                                landmark_model));

		/**
		 * the M_DIST_TH distance from the beginning should be 
		 * n_landmark BEFORE the n_landmarks++ happens in this if statement
		 */
                if (min_ids.at(i) == prev_n_landmarks) // new_landmark (should be prev_n_landmarks?)
                {
                    values.insert(L(n_landmarks), Pose2(global_cone_x(i),
                                                        global_cone_y(i),
                                                        0));
                    n_landmarks++;
		    
		    isam2.update(graph, values);
		    isam2.update();
		    values.clear();
		    graph.resize(0);
		    
                }
            }
	    RCLCPP_INFO(logger, "updating");
	    isam2.update(graph, values);
	    isam2.update();
	    values.clear();
	    graph.resize(0);

        }
        else if (n_landmarks == 0 && cone_obs.size() > 0)
        {

            /* register all observations as new landmarks */
            for (int i = 0; i < cone_obs.size(); i++)
            {
                if (n_landmarks == 0) /* This is for setting the scale in iSAM2*/
		{
		    static noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Diagonal::Sigmas(NoiseModel);
		    gtsam::PriorFactor<Pose2> prior_factor = gtsam::PriorFactor<Pose2>(L(0),
					    Pose2(cone_obs.at(0).x(), cone_obs.at(0).y(), 0),
				    					prior_model);
			
		    graph.add(prior_factor);
		}

                Pose2 global_cone(global_cone_x(i), global_cone_y(i), 0);
                /* add factor node between pose and landmark node */
                graph.add(BetweenFactor<Pose2>(X(pose_num), L(i),
                            Pose2(cone_obs.at(i).x(), cone_obs.at(i).y(), bearing(i, 0)),
                            landmark_model));
                values.insert(L(i), global_cone);

		isam2.update(graph, values);
		isam2.update();
	        values.clear();
		graph.resize(0);

                n_landmarks++;
            }



        }
	
        auto end = high_resolution_clock::now();

        auto duration = duration_cast<microseconds>(end - start);


        RCLCPP_INFO(logger, "DATA ASSOCIATION END; TIME: %d \n", duration.count());

        // DATA ASSOCIATION END

        //Print to squirrel.txt
        //Every single time you run step, you reprint the isam2 estimates
        //Remove the previous estimates (trunc)
        //RCLCPP_INFO(logger, "graphing\n");
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
        robot_est = isam2.calculateEstimate().at(X(pose_num)).cast<gtsam::Pose2>();
        pose_num++;
        // std::cout << "Robot Estimate:(" << robot_est.x() <<"," << robot_est.y() << ")" << std::endl;
    }
};
