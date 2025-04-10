#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include "gtsam/geometry/Pose2.h"
#include "isam2_pkg.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>

using namespace std;
using namespace gtsam;

// const string SAVED_DATA_FILE = "src/isam2/saved_data/vector_val_err_step_input.txt"; // VectorValues error failing test case
const string SAVED_DATA_FILE = "src/isam2/saved_data/SLAM_2lap_step_input.txt";

void parse_pose(string numbers);
void parse_point(string numbers);
void parse_variable_name(string variable_name, string remainder, vector<Point2> &cone_obs);
void read_inputs_from_file(vector<Pose2> &all_poses, vector<Pose2> &all_velocities,
                            vector<double> &all_dt, vector<vector<Point2>> &all_cone_obs);

void print_all_poses(vector<Pose2> &all_poses);
void print_all_cones(vector<vector<Point2>> &all_cone_obs);

#endif
