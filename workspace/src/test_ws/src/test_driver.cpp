#include "test_utils.hpp"

int main() {
    /* Parsing parameters */
    vector<Pose2> all_poses = {};
    vector<Pose2> all_velocities = {};
    vector<double> all_dt = {};
    vector<vector<Point2>> all_cone_obs = {};
    read_inputs_from_file(all_poses, all_velocities, all_dt, all_cone_obs);

    vector<Point2> cone_obs_blue = {};
    vector<Point2> cone_obs_yellow = {};
    vector<Point2> cone_obs_orange = {};

    /* Print to validate parameters */
    cout << "PRINTING" << endl;
    print_all_cones(all_cone_obs);
    print_all_poses(all_poses);

    /* Perform the test */
    slamISAM slam_instance = slamISAM(std::nullopt);
    for (size_t i = 0; i < all_poses.size(); i++) {
        slam_instance.step(all_poses.at(i), all_cone_obs.at(i),
                            cone_obs_blue, cone_obs_yellow, cone_obs_orange,
                            all_velocities.at(i), all_dt.at(i));
        this_thread::sleep_for(chrono::milliseconds(100));
        cout << "Finished step iteration: " << i << endl;
        slam_instance.print_estimates();
    }



    return 0;
}
