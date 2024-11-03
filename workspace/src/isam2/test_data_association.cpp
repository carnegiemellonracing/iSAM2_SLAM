#include <vector>
#include <tuple>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <Eigen/Dense>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <algorithm>
#include <random>
#include <chrono>
#include <float.h>
#include "ros_utils.cpp"


using namespace Eigen;
using namespace std;
using namespace gtsam;
using namespace std::chrono;

void data_association_1(int n_obs, MatrixXd &global_cone_x, MatrixXd &global_cone_y, vector<double> &m_dist,
                        vector<Point2> &cone_obs, vector<Point2> &slam_est, vector<MatrixXd> &slam_mcov)
{
    for (int i = 0; i < n_obs; i++)
    {
        for (int j = 0; j < slam_est.size(); j++)
        {
            MatrixXd diff(1, 2);
            diff << global_cone_x(i, 0) - slam_est.at(j).x(),
                global_cone_y(i, 0) - slam_est.at(j).y();
            m_dist.push_back((diff * slam_mcov.at(j) * diff.transpose())(0, 0));
        }
    }
}
void data_association_2(int n_obs, int n_landmarks, MatrixXd &global_cone_x, MatrixXd &global_cone_y, vector<double> &m_dist,
                        vector<Point2> &cone_obs, vector<Point2> &slam_est, vector<MatrixXd> &slam_mcov)
{
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(n_landmarks, n_landmarks * 2);
    Eigen::MatrixXd b(n_landmarks * 2, 2);
    Eigen::MatrixXd c(2, n_landmarks);
    for (int o = 0; o < n_obs; o++)
    {
        for (int i = 0; i < n_landmarks; i++)
        {
            Point2 cur_delta(global_cone_x(o, 0) - slam_est.at(i).x(), global_cone_y(o, 0) - slam_est.at(i).y());
            a(i, i * 2) = cur_delta.x();
            a(i, i * 2 + 1) = cur_delta.y();
            MatrixXd mcov = slam_mcov[i];
            b.block<2, 2>(i * 2, 0) = mcov;
            c(0, i) = cur_delta.x();
            c(1, i) = cur_delta.y();
        }
        MatrixXd dists = a * b * c;
        for (int i = 0; i < n_landmarks; i++)
        {
            m_dist.push_back(dists(i, i));
        }
    }
}
int main()
{
    int n_obs = 20;
    int n_landmarks = 300;
    MatrixXd global_cone_x(n_obs, 1);
    MatrixXd global_cone_y(n_obs, 1);
    vector<double> m_dist_1;
    vector<double> m_dist_2;
    vector<Point2> cone_obs;
    vector<Point2> slam_est;
    vector<MatrixXd> slam_mcov;
    std::uniform_real_distribution<double> distribution(-10.0, 10.0);
    default_random_engine generator;
    for (int i = 0; i < n_landmarks; i++)
    {
        double x = distribution(generator);
        double y = distribution(generator);
        slam_est.push_back(Point2(x, y));
    }
    for (int i = 0; i < n_obs; i++)
    {
        double x = distribution(generator);
        double y = distribution(generator);
        cone_obs.push_back(Point2(x, y));
        global_cone_x(i, 0) = x;
        global_cone_y(i, 0) = y;
    }
    for (int i = 0; i < n_landmarks; i++)
    {
        MatrixXd cur_cov = MatrixXd::Random(2, 2);
        slam_mcov.push_back(cur_cov);
    }
    auto start = high_resolution_clock::now();
    data_association_1(n_obs, global_cone_x, global_cone_y, m_dist_1,
                       cone_obs, slam_est, slam_mcov);
    auto end = high_resolution_clock::now();
    data_association_2(n_obs, n_landmarks, global_cone_x, global_cone_y, m_dist_2,
                       cone_obs, slam_est, slam_mcov);
    auto end2 = high_resolution_clock::now();
    auto d1 = duration_cast<microseconds>(end - start);
    auto d2 = duration_cast<microseconds>(end2 - end);

    cout << d1.count() << " " << d2.count() << endl;


    //for (int i = 0; i < m_dist_1.size(); i++)
    //{
    //    assert(abs(m_dist_1[i] - m_dist_2[i]) < 0.00000001);
    //    cout << m_dist_1[i] << " " << m_dist_2[i] << endl;
    //}
}