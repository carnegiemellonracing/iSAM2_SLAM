#include "test_utils.hpp"

void parse_pose(string numbers, double &x, double &y, double &z) {
    size_t first_space = numbers.find(" ");
    size_t second_space = numbers.find(" ", first_space + 1);
    if (first_space == std::string::npos || second_space == std::string::npos) {
        throw "Invalid Pose";
    }
    x = std::stod(numbers.substr(0, first_space));
    y = std::stod(numbers.substr(first_space + 1, second_space));
    z = std::stod(numbers.substr(second_space + 1));

    // cout << "Pose: " << x << " " << y << " " << z << endl;

}

void parse_point(string numbers, double &x, double &y) {
    size_t first_space = numbers.find(" ");
    if (first_space == std::string::npos) {
        throw "Invalid point";
    }
    x = std::stod(numbers);
    y = std::stod(numbers.substr(first_space + 1));

    // cout << "Point: " << x << " " << y << endl;
    return ;
}


void parse_variable_name(string variable_name, string remainder,
                        vector<Pose2> &all_poses, vector<Pose2> &all_velocities,
                        vector<double> &all_dt, vector<Point2> &cone_obs) {
    if (variable_name == "global_odom") {
        // Parse out the pose
        string numbers = remainder.substr(1); // Ignoring the space at the beginning
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        parse_pose(numbers, x, y, theta);
        all_poses.emplace_back(x, y, theta);

    } else if (variable_name == "velocity") {
        string numbers = remainder.substr(1); // Ignoring the space at the beginning
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        parse_pose(numbers, x, y, theta);
        all_velocities.emplace_back(x, y, theta);

    } else if (variable_name == "dt") {
        string numbers = remainder.substr(1);
        all_dt.push_back(std::stod(numbers));
        // cout << "dt: " << dt << endl;

    } else if (variable_name == "cone_obs") {
        string numbers = remainder.substr(1);
        double x = 0.0;
        double y = 0.0;
        parse_point(numbers, x, y);
        cone_obs.emplace_back(x, y);

    }

}

void read_inputs_from_file(vector<Pose2> &all_poses, vector<Pose2> &all_velocities,
                            vector<double> &all_dt, vector<vector<Point2>> &all_cone_obs) {

    ifstream file(SAVED_DATA_FILE);

    string line;

    vector<Point2> cur_cone_obs = {};

    bool reading_cones = false;
    int counter = 0;
    while (getline(file, line)) {
        size_t colon_idx = line.find(":");
        if (colon_idx != string::npos) { // If there's something there
            string variable_name = line.substr(0, colon_idx);

            if (reading_cones == false && variable_name.find("cone_obs") != std::string::npos) {
                reading_cones = true;
            } else if (reading_cones == true && variable_name.find("cone_obs") == std::string::npos) {
                reading_cones = false;
                all_cone_obs.push_back(cur_cone_obs);
                cur_cone_obs = {};
            }
            string remainder = line.substr(colon_idx + 1);

            parse_variable_name(variable_name, remainder, all_poses, all_velocities, all_dt, cur_cone_obs);


        } else { // we have a new line
            cout << "New time step " << counter << endl;
            counter++;
            continue;
        }
    }

    file.close();
}

void print_all_poses(vector<Pose2> &all_poses) {
    for (size_t i = 0; i < all_poses.size(); i++) {
        Pose2 p = all_poses.at(i);
        cout << "Pose " << i << " | " << p.x() << " " << p.y() << " " << p.theta() << endl;
    }
}

void print_all_cones(vector<vector<Point2>> &all_cone_obs) {
    for (size_t i = 0; i < all_cone_obs.size(); i++) {
        cout << "Timestep " << i << endl;
        vector<Point2> cur_cone_obs = all_cone_obs.at(i);

        for (size_t c = 0; c < cur_cone_obs.size(); c++) {
            Point2 cur_cone = cur_cone_obs.at(c);
            cout << "Cone " << c << " | " << cur_cone.x() << " " << cur_cone.y() << endl;
        }
    }
}
