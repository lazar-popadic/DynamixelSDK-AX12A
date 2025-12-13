#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/action/ax_bulk_move.hpp"
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

using AxBulkMove = dynamixel_sdk_custom_interfaces::action::AxBulkMove;
using GoalHandleAxBulkMove = rclcpp_action::ServerGoalHandle<AxBulkMove>;

class Ax12aBulkNode : public rclcpp::Node
{
  public:
    Ax12aBulkNode() : Node("ax12a_bulk")
    {
        ax_bulk_move_action_server_ = rclcpp_action::create_server<AxBulkMove>(
            this, "ax_bulk_move", std::bind(&Ax12aBulkNode::handle_goal, this, _1, _2),
            std::bind(&Ax12aBulkNode::handle_cancel, this, _1), std::bind(&Ax12aBulkNode::handle_accepted, this, _1));

        portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Open Serial Port
        int dxl_comm_result = portHandler_->openPort();
        if (dxl_comm_result == false)
            RCLCPP_ERROR(rclcpp::get_logger("ax12a_bulk_node"), "Failed to open the port!");
        else
            RCLCPP_INFO(rclcpp::get_logger("ax12a_bulk_node"), "Succeeded to open the port.");

        // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
        dxl_comm_result = portHandler_->setBaudRate(BAUDRATE);
        if (dxl_comm_result == false)
            RCLCPP_ERROR(rclcpp::get_logger("ax12a_bulk_node"), "Failed to set the baudrate!");
        else
            RCLCPP_INFO(rclcpp::get_logger("ax12a_bulk_node"), "Succeeded to set the baudrate.");

        RCLCPP_INFO(this->get_logger(), "AX-12A bulk node running.");
    }

  private:
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    rclcpp_action::Server<AxBulkMove>::SharedPtr ax_bulk_move_action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const AxBulkMove::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAxBulkMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleAxBulkMove> goal_handle)
    {
        std::thread{std::bind(&Ax12aBulkNode::ax_move, this, _1), goal_handle}.detach();
    }

    void ax_move(const std::shared_ptr<GoalHandleAxBulkMove> goal_handle)
    {
        rclcpp::Rate loop_rate(2);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<AxBulkMove::Feedback>();
        auto result = std::make_shared<AxBulkMove::Result>();

        int8_t status = 0;
        uint8_t dxl_error = 0;
        uint32_t position_velocity_ref = 0;
        std::vector<uint8_t> finished_ids;
        
        uint8_t num_servos = goal->id.size();
        feedback->current_position.resize(num_servos);
        feedback->position_error.resize(num_servos);
        feedback->current_velocity.resize(num_servos);

        dynamixel::GroupSyncWrite syncWrite(portHandler_, packetHandler_, ADDR_GOAL_POSITION, 4);
        uint8_t cnt = 0;
        for (auto &id : goal->id)
        {
            RCLCPP_INFO(this->get_logger(), "Moving AX-12A: \nID: [%d], to position: [%d], with velocity: [%d]", id,
                        goal->position[cnt], goal->velocity[cnt]);

            position_velocity_ref = ((uint32_t)(goal->velocity[cnt] << 16) | (uint32_t)(goal->position[cnt]));
            syncWrite.addParam(id, reinterpret_cast<uint8_t *>(&position_velocity_ref));
            cnt++;
        }

        int dxl_comm_result = syncWrite.txPacket();
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
            cnt = 0;
            if (goal_handle->is_canceling())
                status = -2;

            for (auto &id : goal->id)
            {
                if (!(std::find(finished_ids.begin(), finished_ids.end(), id) != finished_ids.end()))
                {
                    uint32_t present_pos_vel;
                    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, ADDR_PRESENT_POSITION,
                                                                    (&present_pos_vel), &dxl_error);

                    uint16_t present_position = (uint16_t)present_pos_vel;
                    uint16_t present_velocity = (uint16_t)(present_pos_vel >> 16) & 0b0000001111111111;
                    uint16_t position_error = present_position > goal->position[cnt]
                                                  ? present_position - goal->position[cnt]
                                                  : goal->position[cnt] - present_position;

                    feedback->current_position[cnt] = present_position;
                    feedback->position_error[cnt] = position_error;
                    feedback->current_velocity[cnt] = present_velocity;

                    if (position_error < goal->position_tolerance[cnt])
                    {
                        finished_ids.push_back(id);
                        RCLCPP_INFO(this->get_logger(), "ID %d finished.", id);
                    }
                    else if (dxl_error != 0)
                    {
                        // TODO: odadi da otprilike izracunas potrebno vreme, pa lupi delay
                        finished_ids.push_back(id);
                        RCLCPP_WARN(this->get_logger(), "ID %d read failed, considering it success.", id);
                    }
                }
                cnt++;
            }

            goal_handle->publish_feedback(feedback);

            if (finished_ids.size() >= goal->id.size())
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ax12aBulkNode>();
    sleep(1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
