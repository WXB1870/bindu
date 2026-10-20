// Read-only TF source probe. Humble rclpy does not expose publisher GID.
// Emit source metadata to stdout; never publish TF, velocity or device commands.
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

int main(int argc, char **argv) {
  if (argc < 4) return 2;
  rclcpp::init(argc,argv);
  try {
    const double duration=std::stod(argv[3]);
    if (!std::isfinite(duration) || duration<=0 || duration>30) return 2;
    auto node=std::make_shared<rclcpp::Node>("navigation_tf_probe");
    auto callback=[](bool fixed, const tf2_msgs::msg::TFMessage::ConstSharedPtr message,
                     const rclcpp::MessageInfo &info) {
      std::ostringstream gid;
      for (const auto value:info.get_rmw_message_info().publisher_gid.data)
        gid<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<unsigned>(value);
      for (const auto &t:message->transforms) {
        std::cout<<"{\"parent\":"<<std::quoted(t.header.frame_id)<<",\"child\":"<<std::quoted(t.child_frame_id)
          <<",\"gid\":"<<std::quoted(gid.str())<<",\"stamp\":"<<std::setprecision(18)
          <<rclcpp::Time(t.header.stamp).seconds()<<",\"static\":"<<(fixed?"true":"false")<<"}\n";
      }
    };
    auto dynamic=node->create_subscription<tf2_msgs::msg::TFMessage>(argv[1],100,
      [callback](tf2_msgs::msg::TFMessage::ConstSharedPtr m,const rclcpp::MessageInfo &i){callback(false,m,i);});
    auto fixed=node->create_subscription<tf2_msgs::msg::TFMessage>(argv[2],rclcpp::QoS(100).transient_local(),
      [callback](tf2_msgs::msg::TFMessage::ConstSharedPtr m,const rclcpp::MessageInfo &i){callback(true,m,i);});
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(duration);
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    while(rclcpp::ok() && std::chrono::steady_clock::now()<end)
      executor.spin_once(std::chrono::milliseconds(20));
    rclcpp::shutdown();return 0;
  } catch(const std::exception &e) {
    std::cerr<<e.what()<<std::endl;
    rclcpp::shutdown();return 1;
  }
}
