#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/action/ax_hybrid_move.hpp"
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

#define EPS_VELOCITY 12

using namespace std::placeholders;

using AxHybridMove = dynamixel_sdk_custom_interfaces::action::AxHybridMove;
using GoalHandleAxHybridMove = rclcpp_action::ServerGoalHandle<AxHybridMove>;

class Ax12aSingleHybridNode : public rclcpp::Node
{
  public:
    Ax12aSingleHybridNode() : Node("ax12a_single_hybrid")
    {
        ax_hybrid_move_action_server_ = rclcpp_action::create_server<AxHybridMove>(
            this, "ax_hybrid_move", std::bind(&Ax12aSingleHybridNode::handle_goal, this, _1, _2),
            std::bind(&Ax12aSingleHybridNode::handle_cancel, this, _1),
            std::bind(&Ax12aSingleHybridNode::handle_accepted, this, _1));

        portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Open Serial Port
        int dxl_comm_result = portHandler_->openPort();
        if (dxl_comm_result == false)
            RCLCPP_ERROR(rclcpp::get_logger("ax12a_single_hybrid_node"), "Failed to open the port!");
        else
            RCLCPP_INFO(rclcpp::get_logger("ax12a_single_hybrid_node"), "Succeeded to open the port.");

        // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
        dxl_comm_result = portHandler_->setBaudRate(BAUDRATE);
        if (dxl_comm_result == false)
            RCLCPP_ERROR(rclcpp::get_logger("ax12a_single_hybrid_node"), "Failed to set the baudrate!");
        else
            RCLCPP_INFO(rclcpp::get_logger("ax12a_single_hybrid_node"), "Succeeded to set the baudrate.");

        setupDynamixel(BROADCAST_ID);

        RCLCPP_INFO(this->get_logger(), "AX-12A single hybrid node running.");
    }

  private:
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    rclcpp_action::Server<AxHybridMove>::SharedPtr ax_hybrid_move_action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const AxHybridMove::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAxHybridMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleAxHybridMove> goal_handle)
    {
        std::thread{std::bind(&Ax12aSingleHybridNode::ax_hybrid_move, this, _1), goal_handle}.detach();
    }

    void ax_hybrid_move(const std::shared_ptr<GoalHandleAxHybridMove> goal_handle)
    {
        uint freq = 10;
        rclcpp::Rate loop_rate(freq);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<AxHybridMove::Feedback>();
        auto result = std::make_shared<AxHybridMove::Result>();

        RCLCPP_INFO(this->get_logger(), "Hybrid moving AX-12A: \nID: [%d], with velocity: [%d]", goal->id,
                    goal->velocity);

        uint8_t cnt_limit = (uint8_t)(goal->zero_time * freq + 1);
        cnt_limit = cnt_limit > 6 ? 6 : cnt_limit;
        uint8_t zero_vel_cnt = 0;
        RCLCPP_INFO(this->get_logger(), "Cnt limit = %d", (int)cnt_limit);
        int8_t status = 0;
        uint8_t dxl_error = 0;
        uint32_t present_pos_vel = 0;
        uint16_t present_position = 0xffff;
        uint16_t present_velocity = 0xffff;
        int16_t position_ref = 0;

        int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, (uint8_t)goal->id, ADDR_MOVING_VELOCITY,
                                                             goal->velocity, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result));
            status = -3;
        }
        else if (dxl_error != 0)
        {
            RCLCPP_WARN(this->get_logger(), "%s", packetHandler_->getRxPacketError(dxl_error));
            status = -4;
        }

        while (status >= 0)
        {
            if (goal_handle->is_canceling())
                status = -2;

            dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, (uint8_t)goal->id, ADDR_PRESENT_POSITION,
                                                            reinterpret_cast<uint32_t *>(&present_pos_vel), &dxl_error);
            present_position = (uint16_t)present_pos_vel;
            present_velocity = (uint16_t)(present_pos_vel >> 16) & 0b0000001111111111;
            feedback->current_position = present_position;
            feedback->current_velocity = present_velocity;
            goal_handle->publish_feedback(feedback);

            position_ref = (int16_t)present_position + goal->delta_pos;
            position_ref = position_ref > 1023 ? 1023 : position_ref;
            position_ref = position_ref < 0 ? 0 : position_ref;

            int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, (uint8_t)goal->id, ADDR_GOAL_POSITION,
                                                                 position_ref, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result));
                status = -3;
            }
            else if (dxl_error != 0)
            {
                RCLCPP_WARN(this->get_logger(), "%s", packetHandler_->getRxPacketError(dxl_error));
                status = -4;
            }

            if (present_velocity < EPS_VELOCITY)
                zero_vel_cnt++;
            else
                zero_vel_cnt = 0;

            if (zero_vel_cnt >= cnt_limit)
                status = -1;

            loop_rate.sleep();
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

    void setupDynamixel(uint8_t dxl_id)
    {
        uint8_t dxl_error = 0;
        // Enable Torque of DYNAMIXEL
        int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ax12a_single_hybrid_node"), "Failed to enable torque.");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("ax12a_single_hybrid_node"), "Succeeded to enable torque.");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ax12aSingleHybridNode>();
    sleep(1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
