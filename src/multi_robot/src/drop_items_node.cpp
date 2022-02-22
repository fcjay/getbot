
#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DropItems : public plansys2::ActionExecutorClient
{
public:
  DropItems()
  : plansys2::ActionExecutorClient("drop_items", 1s)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.5;
      send_feedback(progress_, "Drop items running");

      std::cout << "Dropping item... " << progress_* 100.0 << "%" << std::endl;


    } else {
      finish(true, 1.0, "Drop items completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    // std::cout << "\r\e[K" << std::flush;
    // std::cout << "Cooking cake ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
    // std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DropItems>();

  node->set_parameter(rclcpp::Parameter("action_name", "drop_items"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

