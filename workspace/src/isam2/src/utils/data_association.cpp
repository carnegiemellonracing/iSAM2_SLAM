/**
 * @file data_association.cpp
 * @brief Perform data association in preparation for feeding data to SLAM
 */
#include "data_association.hpp"
namespace data_association_utils {
    /**
     * @brief Distinguishes obsered cones into old and new cones.
     * 
     * @param global_cone_obs The observed cones in the global frame
     * @param cone_obs The observed cones in the local frame
     * @param distances A vector where the ith vector represents 
     *                  mahalanobis distances wrt the ith observed cone
     * 
     * @param n_landmarks The number of landmarks in the SLAM model 
     * @param logger 
     * @return std::pair<std::vector<OldConeInfo>, std::vector<NewConeInfo>>
     */
    std::pair<std::vector<OldConeInfo>, std::vector<NewConeInfo>> get_old_new_cones(
        std::vector<gtsam::Point2> global_cone_obs, 
        std::vector<gtsam::Point2> cone_obs, 
        std::vector<std::vector<double>> distances, 
        std::size_t n_landmarks,
        std::optional<rclcpp::Logger> logger
    ) {
        std::vector<OldConeInfo> old_cones = {};
        std::vector<NewConeInfo> new_cones = {};

        Eigen::MatrixXd bearing = cone_utils::calc_cone_bearing_from_car(cone_obs);

        for (std::size_t i = 0; i < cone_obs.size(); i++) {
            /* Get the mahalanobis distances wrt cone_obs.at(i) */
            std::vector<double> cur_m_dist = distances.at(i);

            /* Get the minimum distance and the index of the minimum distance */
            std::vector<double>::iterator min_dist = min_element(cur_m_dist.begin(), cur_m_dist.end());
            int min_id = distance(cur_m_dist.begin(), min_dist);

            if (min_id == n_landmarks) {
                new_cones.emplace_back(cone_obs.at(i),
                                        bearing(i, 0),
                                        global_cone_obs.at(i));
            } else {
                old_cones.emplace_back(cone_obs.at(i), 
                                        bearing(i, 0), 
                                        min_id);
            }

        }

        return std::make_pair(old_cones, new_cones);
    }

    std::pair<std::vector<OldConeInfo>, std::vector<NewConeInfo>> perform_data_association(
        gtsam::Pose2 cur_pose, 
        const std::vector<gtsam::Point2> &cone_obs, 
        std::optional<rclcpp::Logger> logger,
        slam::SLAMEstAndMCov& slam_est_and_mcov,  
        double m_dist_th, 
        double cone_dist_th
    ) {

        std::vector<double> m_dist = {};
        std::size_t n_landmarks = slam_est_and_mcov.get_n_landmarks();


        std::vector< gtsam::Point2> relevant_cone_obs = cone_utils::remove_far_cones(cone_obs, cone_dist_th);

        std::vector<gtsam::Point2> global_cone_obs = cone_utils::local_to_global_frame(relevant_cone_obs, cur_pose);
        
        /* A vector where the ith element represents the distances associated with the ith obs cone*/
        std::vector<std::vector<double>> distances = {};
        for (gtsam::Point2 cone : global_cone_obs) {
            distances.push_back(slam_est_and_mcov.calculate_mdist(cone));
        }

        return get_old_new_cones(global_cone_obs, cone_obs, distances, slam_est_and_mcov.get_n_landmarks(), logger);
    }


}

