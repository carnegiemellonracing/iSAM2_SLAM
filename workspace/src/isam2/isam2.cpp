/**@file isam2.cpp
 */

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
static const float M_DIST_TH = 20;
// static const float M_DIST_TH = 45; // used to be 45 lmao
static const long SEC_TO_NANOSEC = 1000000000;
//static mutex global_obs_cones_mutex;
static mutex isam2_mutex;

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
     * @brief Prints the positions of the
     *        observed cones stored within the vector cone_obs.
     *        For DEBUGGING purposes
     *
     * @param cone_obs  A memory address to the vector
     *                  containing the Point2 positions of the observed cones
     *                  (You can just pass in the variable, but the type is
     *                  std::vector<Point2> &)
     *
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
     * @brief   A function called by threads that will
     *          perform data association on the observed Point2 cones stored
     *          inside vector cone_obs.
     *          All calculated mahalanobis distances will be stored in vector
     *          m_dist. For a given time stamp, multiple threads will be spawned
     *          to calculate a subsection of the the mahalanobis distances
     *          from indices [lo, hi)
     *
     * @param global_obs_cones A vector containing the global position
     *                         for the Point2 cones observed in the
     *                         current time stamp
     *
     * @param all_cone_est A vector containing the estimates for the poses
     *                     of previously seen cones, calculated by the
     *                     iSAM2 model
     *
     * @param global_odom A Pose2 variable representing the global pose
     *                    of the car.
     *
     * @param m_dist A vector containing all the mahalanobis distance
     *               calculations that will need to be calculated for the
     *               current time stamp.
     *               Let n_landmarks represent the number of previously
     *               seen landmarks, m_dist is organized such that each
     *               set/multiple of n_landmarks + 1 elements corresponds to
     *               the mahalanobis distance between an observed cone and
     *               the global positions of all previously seen cones. The +1
     *               represents the mahalanobis distance threshold (M_DIST_TH)
     *               These mahalanobis distances will be used to perform
     *               data association on the observed landmarks, by observing
     *               which previously seen cone had the smallest mahalanobis
     *               distance from the current observed cone. If the smallest
     *               distance is M_DIST_TH, then the current observed cone is
     *               a new cone
     *
     * @param lo An integer representing the index in m_dist the current
     *           thread should start populating m_dist
     *
     * @param hi An integer representing the index in m_dist the current
     *           thread should stop populating m_dist. Does not calculate
     *           the element at index hi
     */
    void t_associate(vector<Point2> *cone_obs, vector<Pose2> *global_obs_cones,
            vector<Pose2> *all_cone_est, Pose2 global_odom, vector<float> *m_dist,
            int lo, int hi)
    {
        int i = lo;
        int landmark_idx = lo % (n_landmarks + 1);
        int obs_id = (int)(lo / (n_landmarks + 1));
        //RCLCPP_INFO(logger, "lo: %d | hi: %d", lo, hi);

        while (i < hi && obs_id < cone_obs->size())
        {

            double bearing = std::atan2(cone_obs->at(obs_id).y(),
                                        cone_obs->at(obs_id).x());

            double range = norm2(cone_obs->at(obs_id));

            double global_cone_x = (global_odom.x() + range *
                                            cos(bearing + global_odom.theta()));

            double global_cone_y = (global_odom.y() + range *
                                            sin(bearing + global_odom.theta()));
            /* TODO: "valid bit" for whether global pose has already been calculated at obs_id */

            //TODO: move this outside of t_associate, and before threads are spawned
            global_obs_cones->at(obs_id) = Pose2(global_cone_x, global_cone_y, bearing);


            /**
             * calculate for how many previous cone estimates to calculate
             * mahalanobis distance for, with respect to the current observation.
             *
             * This is because the thread starts calculating mahalanobis distances
             * starting at index lo, which may not be a multiple of n_landmarks + 1
             *
             * (obs_id + 1): which multiple of (n_landmarks +1) depends on which
             *                observed cone
             *
             * (n_landmarks + 1): n_landmarks for all previous landmarks and
             *                    +1 for M_DIST_TH.
             */
            int last_est_for_cur_obs = (obs_id + 1) * (n_landmarks + 1) - 1;


            /* calculating the mahalanobis distances */
            for (; i < last_est_for_cur_obs && i < hi; i++)
            {
                /**
                 * calculate difference between pose of current observed cone
                 * and previously seen cone
                 */
                Eigen::MatrixXd diff(1, 3);
                Pose2 prev_est = all_cone_est->at(landmark_idx);
                diff << global_cone_x - prev_est.x(),
                        global_cone_y - prev_est.y(),
                        1;

                /* calculate the mahalanobis distance */
                m_dist->at(i) = (diff * isam2.marginalCovariance(L(landmark_idx)) * diff.transpose())(0, 0);

                landmark_idx++;
            }

            if (i == hi)
            {
                return;
            }

            m_dist->at(last_est_for_cur_obs) = M_DIST_TH;
            i++; /* skip 1 element (reserved for M_DIST_TH) */
            obs_id++;

            /* new multiple of (n_landmarks+1) means calculating for previous cone/landmark 0 */
            landmark_idx = 0;
        }

    }


    /**
     * @brief   Updates iSAM2 model by first performing data association on
     *          the cones observed from current time stamp. After
     *          determining which of the observed cones are new cones, the
     *          new cones are added to the factor graph which will be used
     *          to update the iSAM2 model.
     *
     * @param logger            RCLCPP logger used to print information to the
     *                          terminal
     *
     * @param global_odom       Pose2 type representing the global pose of the
     *                          car as predicted by the motion model. This will
     *                          be updated by step to hold the global pose of
     *                          the car after the current time step.
     *
     * @param cone_obs          The memory address of the vector containing the
     *                          relative positions (with respect to the car) of
     *                          the cones observed at the current time step.
     *
     * @param orange_ref_cones  The memory address of the vector containing the
     *                          positions of the orange cones observed at the
     *                          start of the track. Will be used for loop
     *                          closure
     *
     * @param velocity          The velocity of the car at the current time
     *                          step. Used as a part of the motion model
     *                          to calculate the current pose of the car.
     *
     * @param time_ns           The time elapsed from the previous time step.
     *                          Used as a part of the motion model to
     *                          calculate the current pose of the car.
     *
     * @param loopClosure       Boolean that will be updated if loop closure
     *                          has been detected.
     */
    void step(auto logger, gtsam::Pose2 global_odom, std::vector<Point2> &cone_obs,
            vector<Point2> &orange_ref_cones, gtsam::Point2 velocity,
            long time_ns, bool loopClosure) {

        RCLCPP_INFO(logger, "stepping\n");


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
        static auto landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);

        if (pose_num==0) { /* first pose */
            static noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Diagonal::Sigmas(NoiseModel);
            gtsam::PriorFactor<Pose2> prior_factor = gtsam::PriorFactor<Pose2>(X(0), global_odom, prior_model);

            /* Add prior (This is how the iSAM2 factor graph begins) */
            graph.add(prior_factor);
            values.insert(X(0), global_odom);


            //ASSUMES THAT YOU SEE ORANGE CONES ON YOUR FIRST MEASUREMENT OF LANDMARKS
            //Add orange cone left and right
            //hopefully it's only 2 cones
            orange_cones = orange_ref_cones;
        }
        else {
            /**
             * Use the previous pose estimate from iSAM2, and velocity odometry
             * info to calculate the current pose
             *
             * Using a velocity-based motion model
             *
             */
            static noiseModel::Diagonal::shared_ptr odom_model = noiseModel::Diagonal::Sigmas(NoiseModel);
            Pose2 prev_pos = isam2.calculateEstimate(X(pose_num - 1)).cast<Pose2>();

            double time_s = time_ns/SEC_TO_NANOSEC;


            Pose2 Odometry =  Pose2(velocity.x()*time_s,
                                    velocity.y()*time_s,
                                    global_odom.theta() - prev_pos.theta());

            /*real data motion model?
            Pose2 Odometry =  Pose2(global_odom.x() - prev_pos.x(),global_odom.y() - prev_pos.y(),
                                    global_odom.theta() - prev_pos.theta());
                                    */

            static noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Diagonal::Sigmas(NoiseModel);
            gtsam::PriorFactor<Pose2> prior_factor = gtsam::PriorFactor<Pose2>(X(0), global_odom, prior_model);
            graph.add(prior_factor);

            gtsam::BetweenFactor<Pose2> odom_factor = gtsam::BetweenFactor<Pose2>(X(pose_num - 1), X(pose_num),
                                                                                    Odometry, odom_model);
            graph.add(odom_factor);
            values.insert(X(pose_num), global_odom);
        }


        if(loopClosure){
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

            gtsam::BetweenFactor<Pose2> odom_factor = gtsam::BetweenFactor<Pose2>(X(0), X(pose_num),AvgPoseDiff, loop_closure_model);
            graph.add(odom_factor);
            values.insert(X(pose_num), global_odom);
        }

        //todo only do this once after update
        isam2.update(graph, values);
        graph.resize(0);
        values.clear();

        RCLCPP_INFO(logger, "DATA ASSOCIATION BEGIN: Pose: (%f, %f)", global_odom.x(),
                                                                        global_odom.y());
        // DATA ASSOCIATION BEGIN

        RCLCPP_INFO(logger, "Printing global obs: (n_landmarks: %d; num_obs: %d)",
                                                    n_landmarks, cone_obs.size());

        auto start = high_resolution_clock::now();
        if (n_landmarks != 0 && cone_obs.size() > 0)
        {
            const int num_threads = 3;
            thread all_t[num_threads];
            const int m_dist_len = (n_landmarks + 1) * cone_obs.size(); //110
            const int multiple_size = m_dist_len / num_threads; //110/3
            vector<float> m_dist(m_dist_len, 0.0);
            vector<Pose2> global_obs_cones = vector(cone_obs.size(), Pose2(0, 0, 0));

            /**
             * Spawn num_threads that will are responsible for performing data
             * association on subset of the observed cones
             *
             * Populate m_dist
             */

            /**
             * Why are we obtaining all the estimates for the global position
             * of previously seen cones sequentially instead of using threads?
             *
             * a.) When using threads, we will have to calculate the estimate
             *     for the same previously seen cone multiple times, for each
             *     observed cone. (wasted computation)
             *
             * b.) Issue with multiple threads calculating estimates at once.
             *     This is a critical section in the code. Therefore, it is
             *     safer to calculate all estimates in a vector, where multiple
             *     threads can safely read from the the vector (even if reading
             *     the same element)
             *
             */
            vector<Pose2> all_cone_est = vector(n_landmarks, Pose2(0, 0, 0));
            for (int i = 0; i < n_landmarks; i++)
            {
                all_cone_est.at(i) = isam2.calculateEstimate(L(i)).cast<Pose2>();
            }


            auto end_temp = high_resolution_clock::now();
            auto duration_temp = duration_cast<microseconds>(end_temp - start);
            RCLCPP_INFO(logger, "temp time: %d", duration_temp.count());

            for (int i = 0; i < num_threads; i++)
            {
                all_t[i] = thread(&slamISAM::t_associate, this, &cone_obs, &global_obs_cones,
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

            t_associate(&cone_obs, &global_obs_cones, &all_cone_est, global_odom, &m_dist,
                                    num_threads * multiple_size, m_dist_len);
            RCLCPP_INFO(logger, "finished populating m_dist\n");

            /**
             * Find min_dist; do this for each (n_landmarks + 1) sub-section of m_dist
             *
             * Finding the previously seen cone that is the most similar to the
             * current observed cone by finding the previously seen cone that
             * had the smallest mahalanobis distance with the observed cone.
             *
             * If the smallest mahalanobis distance is M_DIST_TH (the last
             * element in each (n_landmarks + 1) subsection of m_dist), then
             * the current landmark/cone is a new cone
             *
             */

            for (int i = 0; i < (int)cone_obs.size(); i++)
            {
                int start_idx = i*(n_landmarks + 1);
                vector<float>::iterator start_iter = m_dist.begin() + start_idx;
                int min_id = std::distance(start_iter,
                                        std::min_element(start_iter,
                                                   start_iter + (n_landmarks + 1)));


                graph.add(BetweenFactor<Pose2>(X(pose_num), L(min_id),
                            Pose2(cone_obs.at(i).x(), cone_obs.at(i).y(), global_obs_cones.at(i).theta()),
                            landmark_model));

                if (n_landmarks == min_id) /* new landmark */
                {
                    values.insert(L(n_landmarks), Pose2(global_obs_cones.at(i).x(), global_obs_cones.at(i).y(), 0));
                    n_landmarks++;
                    RCLCPP_INFO(logger, "n_lm: %d", n_landmarks);
                }
            }

        }
        else if (n_landmarks == 0 && cone_obs.size() > 0)
        {

            /* register all observations as new landmarks */
            for (int i = 0; i < cone_obs.size(); i++)
            {
                double bearing = std::atan2(cone_obs.at(i).y(), cone_obs.at(i).x());
                double range = norm2(cone_obs.at(i));
                double global_cone_x = global_odom.x() + range * cos(bearing + global_odom.theta());
                double global_cone_y = global_odom.y() + range * sin(bearing + global_odom.theta());

                Pose2 global_cone(global_cone_x, global_cone_y, 0);
                /* add factor node between pose and landmark node */
                graph.add(BetweenFactor<Pose2>(X(pose_num), L(i),
                            Pose2(cone_obs.at(i).x(), cone_obs.at(i).y(), bearing),
                            landmark_model));
                values.insert(L(i), global_cone);
                n_landmarks++;
            }


            graph.addPrior(L(0),Pose2(cone_obs.at(0).x(), cone_obs.at(0).y(), 0));

        }
        isam2.update(graph, values);
        graph.resize(0);
        values.clear();

        auto end = high_resolution_clock::now();

        auto duration = duration_cast<microseconds>(end - start);


        RCLCPP_INFO(logger, "DATA ASSOCIATION END; TIME: %d \n", duration.count());

        // DATA ASSOCIATION END

        /**
         * Print to squirrel.txt
         * Every single time you run step, you reprint the isam2 estimates
         * Remove the previous estimates (trunc)
         * RCLCPP_INFO(logger, "graphing\n");
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
        robot_est = isam2.calculateEstimate().at(X(pose_num)).cast<gtsam::Pose2>();
        pose_num++;
    }
};
