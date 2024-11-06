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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use Projection factors to model the
// camera's landmark observations. Also, we will initialize the robot at some
// location using a Prior factor.
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

#include "data_association.cpp"
#include "unary_factor.cpp"

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


static const long SEC_TO_NANOSEC = 1000000000;
static const int HEURISTIC_N = 10;
static vector<int> blue_cone_IDs;
static vector<int> yellow_cone_IDs;


class slamISAM {

private:
    ISAM2Params parameters;
    ISAM2 isam2;
    //Create a factor graph and values for new data
    NonlinearFactorGraph graph;
    Values values;

    int pose_num = 0;
    bool first_pose_added = false;

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

    gtsam::Vector LandmarkNoiseModel;
    noiseModel::Diagonal::shared_ptr landmark_model;
    gtsam::Vector PriorNoiseModel;
    noiseModel::Diagonal::shared_ptr prior_model;
    gtsam::Vector OdomNoiseModel;
    noiseModel::Diagonal::shared_ptr odom_model;
    gtsam::Vector UnaryNoiseModel;
    noiseModel::Diagonal::shared_ptr unary_model;


    slamISAM(rclcpp::Logger logger) {
        parameters = ISAM2Params(ISAM2DoglegParams(),0.1,10,true);
        parameters.setFactorization("QR");
        parameters.enablePartialRelinearizationCheck = true;

        isam2 = gtsam::ISAM2(parameters);
        graph = gtsam::NonlinearFactorGraph();
        values = gtsam::Values();
        pose_num = 0;
        first_pose_added = false;
        n_landmarks = 0;
        landmark_est = std::vector<gtsam::Pose2>();

        orange_cones = std::vector<Point2>();

        LandmarkNoiseModel = gtsam::Vector(2);
        // used to be 0.01 for real data
        // 0 for EUFS_SIM
        //TODO: have a different noise model at the beginning
        LandmarkNoiseModel(0) = 0;
        LandmarkNoiseModel(1) = 0;
        landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);

        // used to be all 0s for EUFS_SIM
        PriorNoiseModel = gtsam::Vector(3);
        PriorNoiseModel(0) = 0;
        PriorNoiseModel(1) = 0;
        PriorNoiseModel(2) = 0;

        prior_model = noiseModel::Diagonal::Sigmas(PriorNoiseModel);

        /* Go from 1 pose to another pose*/
        OdomNoiseModel = gtsam::Vector(3);
        OdomNoiseModel(0) = 0;
        OdomNoiseModel(1) = 0;
        OdomNoiseModel(2) = 0;
        odom_model = noiseModel::Diagonal::Sigmas(OdomNoiseModel);

        UnaryNoiseModel = gtsam::Vector(2);
        UnaryNoiseModel(0) = 0;
        UnaryNoiseModel(1) = 0;
        // UnaryNoiseModel(2) = 0.5;
        unary_model = noiseModel::Diagonal::Sigmas(UnaryNoiseModel);




    }

    void update_poses(Pose2 &cur_pose, Pose2 &prev_pose, Pose2 &global_odom,
            Point2 &velocity,double dt, bool new_gps, rclcpp::Logger &logger) {
        /* Adding poses to the SLAM factor graph */
        if (pose_num == 0)
        {
            RCLCPP_INFO(logger, "Processing first pose");
            gtsam::PriorFactor<Pose2> prior_factor = gtsam::PriorFactor<Pose2>(X(0),
                                                                global_odom, prior_model);
            //add prior
            //TODO: need to record the initial bearing because it could be erroneous
            graph.add(prior_factor);
            values.insert(X(0), Pose2(0, 0, global_odom.theta()));

            cur_pose = Pose2(0, 0, global_odom.theta());

                first_pose_added = true;

            //ASSUMES THAT YOU SEE ORANGE CONES ON YOUR FIRST MEASUREMENT OF LANDMARKS
            //Add orange cone left and right
            //hopefully it's only 2 cones
            RCLCPP_INFO(logger, "Finished processing first pose");
        }
        else
        {


            //create a factor between current and previous robot pose
            //add odometry estimates
            //Motion model
            Pose2 new_pose = Pose2(0, 0, 0);
            Pose2 odometry = Pose2(0, 0, 0);

            prev_pose = isam2.calculateEstimate(X(pose_num - 1)).cast<Pose2>();

            //global_odom holds our GPS measurements
            velocity_motion_model(new_pose, odometry, velocity, dt, prev_pose, global_odom);
            RCLCPP_INFO(logger, "GPS position; x: %.10f | y: %.10f", new_pose.x(), new_pose.y());

            RCLCPP_INFO(logger, "Finished motion model");


            gtsam::BetweenFactor<Pose2> odom_factor = BetweenFactor<Pose2>(X(pose_num - 1),
                                                                        X(pose_num),
                                                                            odometry,
                                                                            odom_model);
            cur_pose = Pose2(new_pose.x(), new_pose.y(), new_pose.theta());
            graph.add(odom_factor);
            graph.emplace_shared<UnaryFactor>(X(pose_num), global_odom, unary_model);
            values.insert(X(pose_num), new_pose);

        }


        isam2.update(graph, values);
        graph.resize(0);
        values.clear();


        // Pose2 est_pose = isam2.calculateEstimate(X(pose_num)).cast<Pose2>(); // Safe for pose_num == 0
        // RCLCPP_INFO(logger, "Diff: x: %.10f | y: %.10f", est_pose.x() - cur_pose.x(), est_pose.y() - cur_pose.y());


    }

    /* Cones represented by a tuple: the 1st element is the relative position
     * to the car
     * 2nd Point2 for new_cones represents the global position
     */
    void update_landmarks(vector<tuple<Point2, double, int>> &old_cones,
                        vector<tuple<Point2, double, Point2>> &new_cones,
                        Pose2 &cur_pose,rclcpp::Logger logger) {

        /* Bearing range factor will need
         * Types for car pose to landmark node (Pose2, Point2)
         * Bearing of type Rot2 (Rot2 fromAngle)
         * Range of type double
         * Look at PlanarSLAM example in gtsam
         *
         * When adding values:
         * insert Point2 for the cones and their actual location
         *
         */
        for (int o = 0; o < old_cones.size(); o++) {
            Point2 cone_car_frame = get<0>(old_cones.at(o));
            int min_id = get<2>(old_cones.at(o));
            Rot2 b = Rot2::fromAngle(get<1>(old_cones.at(o)));
            double r = norm2(get<0>(old_cones.at(o)));

            graph.add(BearingRangeFactor<Pose2, Point2>(X(pose_num), L(min_id),
                                            b,
                                            r,
                                            landmark_model));
        }
        isam2.update(graph, values);
        graph.resize(0);
        // values should be empty

        for (int n = 0; n < new_cones.size(); n++) {
            Point2 cone_car_frame = get<0>(new_cones.at(n));
            Rot2 b = Rot2::fromAngle(get<1>(new_cones.at(n)));
            double r = norm2(get<0>(new_cones.at(n)));
            Point2 cone_global_frame = get<2>(new_cones.at(n));

            graph.add(BearingRangeFactor<Pose2, Point2>(X(pose_num), L(n_landmarks),
                                            b,
                                            r,
                                            landmark_model));

            values.insert(L(n_landmarks), cone_global_frame);
            n_landmarks++;
        }
        /* All values in graph must be in values parameter */
        if (n_landmarks < 30) {
            values.insert(X(pose_num), cur_pose);
            Values optimized_val = LevenbergMarquardtOptimizer(graph, values).optimize();
            optimized_val.erase(X(pose_num));
            isam2.update(graph, optimized_val);

        } else {
            isam2.update(graph, values);
        }
        graph.resize(0); //Not resizing your graph will result in long update times
        values.clear();

        RCLCPP_INFO(logger, "n_landmarks: %d", n_landmarks);
    }


    /**
     * @brief Obtains information about the observed cones from the current time
     *        step as well as odometry information (to use motion model to
     *        calculate current pose).
     */
    void step(rclcpp::Logger logger, gtsam::Pose2 global_odom, vector<Point2> &cone_obs,
                vector<Point2> &cone_obs_blue, vector<Point2> &cone_obs_yellow,
                vector<Point2> &orange_ref_cones, gtsam::Point2 velocity,
                double dt) {

        if (n_landmarks > 0)
        {
            auto end = high_resolution_clock::now();
            auto d = duration_cast<microseconds>(end - start);
            RCLCPP_INFO(logger, "Step time: %d", d.count());
        }

        start = high_resolution_clock::now();

        Pose2 cur_pose = Pose2(0, 0, 0);
        Pose2 prev_pose = Pose2(0, 0, 0);
        auto start_update_poses = high_resolution_clock::now();
        update_poses(cur_pose, prev_pose, global_odom, velocity, dt, false,logger);
        auto end_update_poses = high_resolution_clock::now();
        auto dur_update_poses = duration_cast<microseconds>(end_update_poses - start_update_poses);
        RCLCPP_INFO(logger, "update_poses time: %d", dur_update_poses.count());

        auto start_est_retrieval = high_resolution_clock::now();
        vector<Point2> slam_est = {};
        for (int i = 0; i < n_landmarks; i++) {
            slam_est.push_back(isam2.calculateEstimate(L(i)).cast<Point2>());
        }

        vector<MatrixXd> slam_mcov = {};
        for (int i = 0; i < n_landmarks; i++) {
            slam_mcov.push_back(isam2.marginalCovariance(L(i)));
        }
        auto end_est_retrieval = high_resolution_clock::now();
        auto dur_est_retrieval = duration_cast<microseconds>(end_est_retrieval - start_est_retrieval);
        RCLCPP_INFO(logger, "est_retrieval time: %d", dur_est_retrieval.count());

        vector<tuple<Point2, double, int>> old_cones = {};
        vector<tuple<Point2, double, Point2>> new_cones = {};

        auto start_DA = high_resolution_clock::now();
        data_association(old_cones, new_cones, cur_pose, prev_pose,
                            cone_obs, logger, slam_est, slam_mcov);
        auto end_DA = high_resolution_clock::now();
        auto dur_DA = duration_cast<microseconds>(end_DA - start_DA);
        RCLCPP_INFO(logger, "Data association time: %d", dur_DA.count());

        auto start_update_landmarks = high_resolution_clock::now();
        update_landmarks(old_cones, new_cones, cur_pose, logger);
        auto end_update_landmarks = high_resolution_clock::now();
        auto dur_update_landmarks = duration_cast<microseconds>(end_update_landmarks - start_update_landmarks);
        RCLCPP_INFO(logger, "update_landmarks time: %d", dur_update_landmarks.count());

        pose_num++;
        /* Create a boolean to check if mahalanobis calcs are done for current
         * time step before proceeding to mahalanobis calcs for next time step
         *
         */

        auto start_vis_setup = high_resolution_clock::now();
        std::ofstream ofs;
        std::ofstream out("squirrel.txt");
        std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
        std::cout.rdbuf(out.rdbuf());
        ofs.open("squirrel.txt", std::ofstream::out | std::ofstream::trunc);
        auto estimate = isam2.calculateEstimate();
        estimate.print("Estimate:");
        ofs.close();
        std::cout.rdbuf(coutbuf); //reset to standard output again
        auto end_vis_setup = high_resolution_clock::now();
        auto dur_vis_setup = duration_cast<microseconds>(end_vis_setup - start_vis_setup);
        RCLCPP_INFO(logger, "vis_setup time: %d", dur_vis_setup.count());


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
