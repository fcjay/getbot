
#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ActionController : public rclcpp::Node
{
public:
  ActionController()
  : rclcpp::Node("action_controller")
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    init_knowledge();

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }

    //if (!executor_client_->start_plan_execution(plan.value())) {
     // RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
   // }

    //return true;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"w1", "world"});
    problem_expert_->addInstance(plansys2::Instance{"r1", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"b1", "balls"});

    problem_expert_->addPredicate(plansys2::Predicate("(balls_dropped r1 b1)"));

    problem_expert_->addFunction(plansys2::Function("= battery_level r1 100"));
    problem_expert_->addFunction(plansys2::Function("= detected_balls b1 5"));
    problem_expert_->addFunction(plansys2::Function("= at_target_balls b1 0"));
    problem_expert_->addFunction(plansys2::Function("= carried_balls r1 0"));
    problem_expert_->addFunction(plansys2::Function("= at_target_balls_goal b1 2"));

    problem_expert_->setGoal(plansys2::Goal("(and(balls_handeled r1 b1))"));
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionController>();

  if (!node->init()) {
    return 0;
  }

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}