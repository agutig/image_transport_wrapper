


void calculate_bitrate(size_t message_size_bits, double& last_bitrate, rclcpp::Time& last_message_time) {
    auto now = this->now();
    auto time_diff = now - last_message_time;
    auto time_diff_sec = time_diff.seconds() + time_diff.nanoseconds() / 1e9;

    if (time_diff_sec > 0) {
        last_bitrate = message_size_bits / time_diff_sec / 1e6; // Mbits/sec
    }

    last_message_time = now;
}