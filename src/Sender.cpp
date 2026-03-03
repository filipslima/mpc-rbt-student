#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{
  //UNIMPLEMENTED(__PRETTY_FUNCTION__);

  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  data.x += 1.0;
  data.y += 1.0;
  data.z += 1.0;
  data.frame = "sender_frame";

  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };

  if (!Utils::Message::serialize(frame, data)) {
    RCLCPP_WARN(logger, "Failed to serialize data.");
    return;
  }

  send(frame);

  RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);

  RCLCPP_INFO(
    logger, "\n\tstamp: %ld\n\tx: %f\n\ty: %f\n\tz: %f", data.timestamp, data.x, data.y, data.z);
}