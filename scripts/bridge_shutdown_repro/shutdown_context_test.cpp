// Synthetic exception-policy regression; never baseline qualification evidence.
#define main parameter_bridge_main
#include BRIDGE_MAIN
#undef main
#include <chrono>

int main(int argc, char **argv)
{
  for (int scenario=0; scenario<3; ++scenario) {
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("synthetic_shutdown_context_test");
    bool invoked=false, caught=false;
    auto timer=node->create_wall_timer(std::chrono::milliseconds(1),[&]() {
      invoked=true;
      if (scenario==0) {
        rclcpp::shutdown();
        // Real Humble graph query after context shutdown, matching the defect.
        node->get_subscriptions_info_by_topic("/synthetic_read_only_query");
        throw std::runtime_error("graph query unexpectedly accepted invalid context");
      }
      if (scenario==2) rclcpp::shutdown();
      rcl_error_state_t state{};
      throw rclcpp::exceptions::RCLError(
        scenario==1 ? RCL_RET_NODE_INVALID : RCL_RET_ERROR,&state,"synthetic policy test");
    });
    try {spin_bridge_node(node);}
    catch (const rclcpp::exceptions::RCLError &) {caught=true;}
    rclcpp::shutdown();
    if (!invoked || caught!=(scenario!=0)) return 1;
  }
  std::cout << "3 synthetic shutdown exception policy cases passed\n";
  return 0;
}
