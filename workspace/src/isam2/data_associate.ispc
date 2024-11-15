export void data_associate_3(uniform int n_obs, uniform int n_landmarks,
                             uniform double* global_cone_x, uniform double* global_cone_y,
                             uniform double* slam_est_x, uniform double* slam_est_y,
                             uniform double* slam_mcov, uniform double* m_dist) {
    uniform int total = n_obs * n_landmarks;
    foreach(idx = 0 ... total) {
        int i = idx / n_landmarks;
        int j = idx % n_landmarks;
        double diff_x = global_cone_x[i] - slam_est_x[j];
        double diff_y = global_cone_y[i] - slam_est_y[j];
        double AB_x = diff_x * slam_mcov[4 * j] + diff_y * slam_mcov[4 * j + 1];
        double AB_y = diff_x * slam_mcov[4 * j + 2] + diff_y * slam_mcov[4 * j + 3];
        double result = AB_x * diff_x + AB_y * diff_y;
        m_dist[idx] = result;
    }
}