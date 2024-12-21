#include "test_utils.hpp"

int main() {
    vector<Pose2> all_poses = {};
    vector<Pose2> all_velocities = {};
    vector<double> all_dt = {};
    vector<vector<Point2>> all_cone_obs = {};
    read_inputs_from_file(all_poses, all_velocities, all_dt, all_cone_obs);

    slamISAM slam_instance = slamISAM(std::nullopt);
    cout << "PRINTING" << endl;
    print_all_cones(all_cone_obs);
    print_all_poses(all_poses);
    return 0;
}