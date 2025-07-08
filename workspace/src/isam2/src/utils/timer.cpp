#include "timer.hpp"

Timer::Timer(size_t batch_size)
	: batch_size_(batch_size), step_count_(0) {}

void Timer::start() {
	start_time_ = std::chrono::high_resolution_clock::now();
}

void Timer::stop_and_record() {
	auto end_time = std::chrono::high_resolution_clock::now();
	double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time_.value()).count();
	recorded_times_.push_back(duration_ms);

	step_count_++;

	if (step_count_ % batch_size_ == 0) {
		double avg_time = compute_batch_average();
		batch_averages_.push_back(avg_time);
		
		std::ofstream ofs(TIMER_FILE, std::ios::app);
		if (ofs.is_open()) {
			ofs << "Batch " << (step_count_ / batch_size_) << ": "
				<< avg_time << " ms" << std::endl;
			ofs.close();
		} else {
			std::cerr << "Error: Could not open " << TIMER_FILE << "." << std::endl;
		}

		recorded_times_.clear();
	}
}

double Timer::compute_batch_average() const {
	if (recorded_times_.empty()) return 0.0;
	double sum = std::accumulate(recorded_times_.begin(), recorded_times_.end(), 0.0);
	return sum / recorded_times_.size();
}

const std::vector<double>& Timer::get_all_batch_averages() const {
	return batch_averages_;
}
