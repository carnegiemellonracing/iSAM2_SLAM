// #include "isam2.hpp"
#include <type_traits>

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

// using namespace std;
using namespace gtsam;

static const int M_DIST_TH = 100;//5.99;

static const float DT = 0.1;
static const float SIM_TIME = 50.0;
static const int LM_SIZE = 2;
static const int STATE_SIZE = 3;

static const int N_STEP = 100;

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
private:
    ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    ISAM2 isam2;
    //Create a factor graph and values for new data
    NonlinearFactorGraph graph;
    Values values;
    Values observed;

    int x;
    //Define empty set
    // using Cmp = std::integral_constant<decltype(&cmp), &cmp>;
    // std::set<Landmark, const Compare> observed;

    gtsam::Symbol X(int robot_pose_id) {
        return Symbol('x', robot_pose_id);
    }

    gtsam::Symbol L(int cone_pose_id) {
        return Symbol('l', cone_pose_id);
    }

public:

    int n_landmarks;

    gtsam::Pose2 robot_est;
    std::vector<gtsam::Point2> landmark_est;

    slamISAM() {

        isam2 = gtsam::ISAM2();
        graph = gtsam::NonlinearFactorGraph();
        values = gtsam::Values();
        observed = gtsam::Values();
        x = 0;
        n_landmarks = 0;
        robot_est = gtsam::Pose2(0, 0, 0);
        landmark_est = std::vector<gtsam::Point2>();

        // isam2.joint1(1,2);
    }

    double mahalanobisDist(Point2 measurement,Point2 landmark,Symbol landmark_key){
        std::cout << "Landmark key:\n"  << landmark_key << std::endl;

        Matrix marginal_covariance = isam2.marginalCovariance(landmark_key);

        std::cout << "Covariance Matrix:\n"  << marginal_covariance << std::endl;

        Eigen::MatrixXd diff(1, 2); 
        diff << measurement.x()-landmark.x(),measurement.y()-landmark.y();

        std::cout << "Diff:\n"  << diff << std::endl;

        Eigen::MatrixXd result = diff*marginal_covariance*diff.transpose();
        
        std::cout << "Mahalanobis result:\n"  << result << std::endl;

        return result(0);
    }

    int associate(Point2 measurement) {
        // Vector that will store mahalanobis distances
        std::vector<double> min_dist;
        for (int i = 0; i < n_landmarks; i++) {
            gtsam::Point2 landmark = observed.at(L(i)).cast<Point2>();
            // Adding mahalanobis distance to minimum distance vector
            std::cout << "Before Mahalanobis key:\n"  << L(i) << std::endl;

            double mahalanobis = mahalanobisDist(measurement,landmark,L(i));
            min_dist.push_back(mahalanobis);
        }
        min_dist.push_back(M_DIST_TH); // Add M_DIST_TH for new landmark
        // Find the index of the minimum element in 'min_dist'
        //min_id will be equal to num_landmarks if it didn't find anything under M_DIST_TH
        int min_id = std::distance(min_dist.begin(), std::min_element(min_dist.begin(), min_dist.end()));
        return min_id;
    }

    void step(gtsam::Pose2 global_odom, std::vector<Point2> &cone_obs) {
        Vector NoiseModel(3);
        NoiseModel(0) = 0;
        NoiseModel(1) = 0;
        NoiseModel(2) = 0;

        Vector LandmarkNoiseModel(2);
        LandmarkNoiseModel(0) = 0;
        LandmarkNoiseModel(1) = 0;

        Pose2 prev_robot_est;
        if (x==0) {//if this is the first pose, add your inital pose to the factor graph
            std::cout << "First pose\n" << std::endl;

            static noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Diagonal::Sigmas(NoiseModel);
            gtsam::PriorFactor<Pose2> prior_factor = gtsam::PriorFactor<Pose2>(X(0), global_odom, prior_model);
            //add prior
            graph.add(prior_factor);
            values.insert(X(0), global_odom);

            prev_robot_est = Pose2(0, 0, 0);
        }
        else {
            std::cout << "New Pose\n" << std::endl;
            static noiseModel::Diagonal::shared_ptr odom_model = noiseModel::Diagonal::Sigmas(NoiseModel);
            Pose2 prev_pos = isam2.calculateEstimate(X(x - 1)).cast<Pose2>();
            //create a factor between current and previous robot pose
            gtsam::BetweenFactor<Pose2> odom_factor = gtsam::BetweenFactor<Pose2>(X(x - 1), X(x), Pose2(global_odom.x() - prev_pos.x(), global_odom.y() - prev_pos.y(), global_odom.theta() - prev_pos.theta()), odom_model);
            graph.add(odom_factor);
            values.insert(X(x), global_odom);
            prev_robot_est = prev_pos;
        }

        //todo only do this once after update
        isam2.update(graph, values);
        graph.resize(0);
        values.clear();

        Pose2 robot_est = isam2.calculateEstimate(X(x)).cast<Pose2>();


        // DATA ASSOCIATION BEGIN
        for (Point2 cone : cone_obs) { // go through each observed cone
            RCLCPP_INFO(logger, "observed cone: (%f,%f)",cone.x(),cone.y());


            // Pose2 conePose(cone.x(),cone.y(),0);
            Point2 global_cone(global_odom.x() + cone.x(), global_odom.y() + cone.y()); //calculate global position of the cone
            // Pose2 global_cone(global_odom.x() + cone.x(), global_odom.y() + cone.y(),0);

            //for the current cone, we want to compare againt all other cones for data association
            //TODO: instead of iterating through all of the landmarks, see if there is a way to do this with a single operation
            //This is jvc lmao
            int associated_ID = associate(global_cone);
            std::cout << "Associated ID:\n"  << associated_ID << std::endl;

            static auto landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);
            //If it is a new cone:
            if (associated_ID == n_landmarks) { //if you can't find it in the list of landmarks
                //add cone to list
                // observed.insert(enum_cone);
                std::cout << "New Landmark:\n"  << L(n_landmarks) << std::endl;

                //add factor between pose and landmark
                double range =  norm2(cone);//std::sqrt(cone.x() * cone.x() + cone.y() * cone.y());
                double bearing = std::atan2(cone.y(), cone.x()) - global_odom.theta();
                Rot2 angle = Rot2(bearing);
      
                graph.add(BearingRangeFactor<Pose2, Point2, Rot2, double>(X(x), L(n_landmarks), angle, range, landmark_model)); 
                //this is how we model noise for the environmant
                std::cout << "added bearing range factor\n"<< std::endl;

                values.insert(L(n_landmarks), global_cone);
                observed.insert(L(n_landmarks), global_cone);


                std::cout << "after inserting values\n"<< std::endl;

                isam2.update(graph, values);
                graph.resize(0);
                values.clear();

                n_landmarks++;
            } else {
                std::cout << "Associated Landmark:\n"  << X(x) << std::endl;

                //Add a factor to the associated landmark
                double range =  norm2(cone);//std::sqrt(cone.x() * cone.x() + cone.y() * cone.y());
                double bearing = std::atan2(cone.y(), cone.x()) - global_odom.theta();
                Rot2 angle = Rot2(bearing);
                graph.add(BearingRangeFactor<Pose2, Point2, Rot2, double>(X(x), L(associated_ID), angle, range, landmark_model));
            }

        }
        // DATA ASSOCIATION END
        isam2.update(graph, values);
        graph.resize(0);
        values.clear();


        //calculate estimate of robot state
        robot_est = isam2.calculateEstimate(X(x)).cast<gtsam::Pose2>();
        x++;

        // landmark_est.clear();
        // for (int i = 0; i < n_landmarks; i++) {

        //     auto landmarkVal = isam2.calculateEstimate(L(i)).cast<gtsam::Point2>();
        //     auto it = find(landmark_est.begin(), landmark_est.end(), landmarkVal); 
        //     if (it != landmark_est.end()) { 
        //         // Pair found 
        //     } else { 
        //         landmark_est.push_back(isam2.calculateEstimate(L(i)).cast<gtsam::Point2>());
        //         // Pair not found 
        //     } 
        // }
    }
};