#pragma once

#include "constants.hpp"

#include <fstream>
#include <iostream>
#include <numeric>
#include <chrono>
#include <vector>
#include <optional>
#include <map>


class Timer {
	public:
		explicit Timer(size_t batch_size = TIMER_BATCH_SIZE);

		void start();
		void stop_and_record(size_t total_cones, size_t pose_num);

		double compute_bucket_average(size_t bucket_index) const;

		void deactivate();

		void print_summary();
		
	private:
		size_t batch_size_;
		std::optional<std::chrono::high_resolution_clock::time_point> start_time_;
		std::map<size_t, std::vector<double>> bucket_times_;
		bool active_ = true;
		bool summary_ = false;
};
