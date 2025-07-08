#pragma once

#include "constants.hpp"

#include <fstream>
#include <iostream>
#include <numeric>
#include <chrono>
#include <vector>
#include <optional>


class Timer {
	public:
		explicit Timer(size_t batch_size = TIMER_BATCH_SIZE);

		void start();
		void stop_and_record();

		double compute_batch_average() const;
		const std::vector<double>& get_all_batch_averages() const;

	private:
		size_t batch_size_;
		size_t step_count_;
		std::optional<std::chrono::high_resolution_clock::time_point> start_time_;
		std::vector<double> recorded_times_;
		std::vector<double> batch_averages_;
};
