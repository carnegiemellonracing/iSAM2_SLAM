#include "timer.hpp"

Timer::Timer(size_t batch_size)
	: batch_size_(batch_size) {
	std::ofstream ofs(TIMER_FILE, std::ios::trunc);
	ofs.close();
}

void Timer::start() {
	if (!active_) return;
	start_time_ = std::chrono::high_resolution_clock::now();
}

void Timer::stop_and_record(size_t total_cones, size_t pose_num) {
	if (!active_ || !start_time_.has_value()) return;

	auto end_time = std::chrono::high_resolution_clock::now();
	double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time_.value()).count();

	size_t bucket_index = (total_cones / batch_size_) + 1;
	bucket_times_[bucket_index].push_back(duration_ms);

	std::ofstream ofs(TIMER_FILE, std::ios::app);
	ofs << "Pose: " << pose_num << " | "
		<< "Bucket: " << bucket_index << " | " 
		<< "Cones: " << total_cones << " | " 
		<< "Time (ms): " << duration_ms << "\n" << std::endl;
}

double Timer::compute_bucket_average(size_t bucket_index) const {
	auto it = bucket_times_.find(bucket_index);
	if (it == bucket_times_.end() || it->second.empty()) return 0.0;

	const std::vector<double>& times = it->second;
	double sum = std::accumulate(times.begin(), times.end(), 0.0);
	return sum / times.size();
}

const std::map<size_t, std::vector<double>>& Timer::get_all_bucket_times() const {
	return bucket_times_;
}

void Timer::deactivate() {
	active_ = false;
}