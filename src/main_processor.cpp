#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class MainProcessorNode : public rclcpp::Node
{
public:
  MainProcessorNode() : Node("main_processor_node")
  {
    RCLCPP_INFO(this->get_logger(), "MainProcessorNode starting...");

    service_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // client_callback_group_ =
    //   this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // NOTE: Deadlock is not occurred
    client_callback_group_ = service_callback_group_;  // NOTE: Deadlock is occurred

    main_service_ = this->create_service<std_srvs::srv::Trigger>(
      "main_trigger",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->handle_main_trigger(request, response);
      },
      rmw_qos_profile_services_default, service_callback_group_);

    sub_client_ = this->create_client<std_srvs::srv::Trigger>(
      "sub_trigger", rmw_qos_profile_services_default, client_callback_group_);

    RCLCPP_INFO(this->get_logger(), "MainProcessorNode initialized successfully");
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr main_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sub_client_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;

  void handle_main_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Main request received");

    if (!sub_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Sub service unavailable");
      response->success = false;
      response->message = "Sub service unavailable";
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sub service available");

    auto sub_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = sub_client_->async_send_request(sub_request);

    auto wait_status = future.wait_for(std::chrono::seconds(1));

    RCLCPP_INFO(this->get_logger(), "Waiting for Sub service response...");

    if (wait_status == std::future_status::ready) {
      auto result = future.get();

      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Main completed successfully");
        response->success = true;
        response->message = "Main completed successfully";
      } else {
        RCLCPP_ERROR(this->get_logger(), "Sub failed: %s", result->message.c_str());
        response->success = false;
        response->message = "Sub failed: " + result->message;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Sub service call failed");
      response->success = false;
      response->message = "Sub service call failed";
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MainProcessorNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
