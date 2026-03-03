#include <mpc-rbt-solution/Receiver.hpp>

void Receiver::Node::run()
{
  while (errno != EINTR) {
    RCLCPP_INFO(logger, "Waiting for data ...");
    Socket::IPFrame frame{};
    if (receive(frame)) {
      RCLCPP_INFO(logger, "Received data from host: '%s:%d'", frame.address.c_str(), frame.port);

      callback(frame);

    } else {
      RCLCPP_WARN(logger, "Failed to receive data.");
    }
  }
}

void Receiver::Node::onDataReceived(const Socket::IPFrame & frame)
{
  //UNIMPLEMENTED(__PRETTY_FUNCTION__);

  if (!Utils::Message::deserialize(frame, data)) {
    RCLCPP_WARN(logger, "Failed to deserialize data.");
    return;
  }

  RCLCPP_INFO(
    logger, "\n\tstamp: %ld\n\tx: %f\n\ty: %f\n\tz: %f", data.timestamp, data.x, data.y, data.z);
}