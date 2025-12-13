#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/action/ax_move.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcutils/cmdline_parser.h"

// Control table address for AX-12A
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_MOVING_VELOCITY 32
#define ADDR_PRESENT_POSITION 36
#define ADDR_PRESENT_VELOCITY 38

#define PROTOCOL_VERSION 1.0

#define BAUDRATE 9600
#define DEVICE_NAME "/dev/ttyUSB0"

using namespace std::placeholders;

using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
using AxMove = dynamixel_sdk_custom_interfaces::action::AxMove;
using GoalHandleAxMove = rclcpp_action::ServerGoalHandle<AxMove>;

class Ax12aSingleNode : public rclcpp::Node
{
  public:
    Ax12aSingleNode() : Node("ax12a_single")
    {
        set_position_subscriber_ = this->create_subscription<SetPosition>(
            "set_position", 10, std::bind(&Ax12aSingleNode::callback_set_position, this, _1));

        get_position_server_ = this->create_service<GetPosition>(
            "get_position", std::bind(&Ax12aSingleNode::callback_get_position, this, _1, _2));

        ax_move_action_server_ = rclcpp_action::create_server<AxMove>(
            this, "ax_move", std::bind(&Ax12aSingleNode::handle_goal, this, _1, _2),
            std::bind(&Ax12aSingleNode::handle_cancel, this, _1),
            std::bind(&Ax12aSingleNode::handle_accepted, this, _1));

        portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Open Serial Port
        int dxl_comm_result = portHandler_->openPort();
        if (dxl_comm_result == false)
            RCLCPP_ERROR(rclcpp::get_logger("ax12a_single_node"), "Failed to open the port!");
        else
            RCLCPP_INFO(rclcpp::get_logger("ax12a_single_node"), "Succeeded to open the port.");

        // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
        dxl_comm_result = portHandler_->setBaudRate(BAUDRATE);
        if (dxl_comm_result == false)
            RCLCPP_ERROR(rclcpp::get_logger("ax12a_single_node"), "Failed to set the baudrate!");
        else
            RCLCPP_INFO(rclcpp::get_logger("ax12a_single_node"), "Succeeded to set the baudrate.");

        RCLCPP_INFO(this->get_logger(), "AX-12A single node running.");
    }

  private:
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
    rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
    rclcpp_action::Server<AxMove>::SharedPtr ax_move_action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const AxMove::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAxMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleAxMove> goal_handle)
    {
        std::thread{std::bind(&Ax12aSingleNode::ax_move, this, _1), goal_handle}.detach();
    }

    void ax_move(const std::shared_ptr<GoalHandleAxMove> goal_handle)
    {
        rclcpp::Rate loop_rate(10);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<AxMove::Feedback>();
        auto result = std::make_shared<AxMove::Result>();

        RCLCPP_INFO(this->get_logger(), "Moving AX-12A: \nID: [%d], to position: [%d], with velocity: [%d]", goal->id,
                    goal->position, goal->velocity);

        int8_t status = 0;
        uint8_t dxl_error = 0;
        uint32_t position_velocity_ref = ((uint32_t)(goal->velocity << 16) | (uint32_t)(goal->position));
        uint32_t present_pos_vel = 0;
        uint16_t present_position = 0xffff;
        uint16_t present_velocity = 0xffff;
        uint16_t position_error = 0xffff;

        int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, (uint8_t)goal->id, ADDR_GOAL_POSITION,
                                                             position_velocity_ref, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result));
            status = -3;
            return;
        }
        else if (dxl_error != 0)
        {
            RCLCPP_WARN(this->get_logger(), "%s", packetHandler_->getRxPacketError(dxl_error));
            status = -4;
            return;
        }

        while (status >= 0)
        {
            if (goal_handle->is_canceling())
                status = -2;

            dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, (uint8_t)goal->id, ADDR_PRESENT_POSITION,
                                                            reinterpret_cast<uint32_t *>(&present_pos_vel), &dxl_error);
            present_position = (uint16_t)present_pos_vel;
            present_velocity = (uint16_t)(present_pos_vel >> 16) & 0b0000001111111111;
            position_error = present_position > goal->position ? present_position - goal->position
                                                               : goal->position - present_position;
            feedback->current_position = present_position;
            feedback->position_error = position_error;
            feedback->current_velocity = present_velocity;
            goal_handle->publish_feedback(feedback);

            if (position_error < goal->position_tolerance)
                status = -1;

            goal->position, loop_rate.sleep();
        }

        result->status = status;
        if (rclcpp::ok())
        {
            switch (result->status)
            {
            case -1:
                RCLCPP_INFO(this->get_logger(), "Move succeeded...");
                break;
            case -2:
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Move canceled...");
                return;
                break;
            }
            goal_handle->succeed(result);
        }
        else
            result->status = -100;
    }

    void callback_set_position(const std::shared_ptr<dynamixel_sdk_custom_interfaces::msg::SetPosition> msg)
    {
        uint8_t dxl_error = 0;
        uint16_t goal_position = (uint16_t)msg->position;

        int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, (uint8_t)msg->id, ADDR_GOAL_POSITION,
                                                             goal_position, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler_->getRxPacketError(dxl_error));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
        }
    }

    void callback_get_position(const std::shared_ptr<GetPosition::Request> request,
                               std::shared_ptr<GetPosition::Response> response)
    {
        uint16_t present_position = 0;
        uint8_t dxl_error = 0;
        int dxl_comm_result =
            packetHandler_->read2ByteTxRx(portHandler_, (uint8_t)request->id, ADDR_PRESENT_POSITION,
                                          reinterpret_cast<uint16_t *>(&present_position), &dxl_error);
        (void)dxl_comm_result;
        RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present Position: %d]", request->id, present_position);

        response->position = present_position;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ax12aSingleNode>();
    sleep(1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
