#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

// Control table address for AX-12A
#define ADDR_TORQUE_ENABLE 24

#define PROTOCOL_VERSION 1.0

#define BAUDRATE 9600
#define DEVICE_NAME "/dev/ttyUSB0"

using namespace std::placeholders;

class Ax12aSetupNode : public rclcpp::Node
{
  public:
    Ax12aSetupNode() : Node("ax12a_setup")
    {
        portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Open Serial Port
        int dxl_comm_result = portHandler_->openPort();
        if (dxl_comm_result == false)
            RCLCPP_ERROR(rclcpp::get_logger("ax12a_setup_node"), "Failed to open the port!");
        else
            RCLCPP_INFO(rclcpp::get_logger("ax12a_setup_node"), "Succeeded to open the port.");

        // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
        dxl_comm_result = portHandler_->setBaudRate(BAUDRATE);
        if (dxl_comm_result == false)
            RCLCPP_ERROR(rclcpp::get_logger("ax12a_setup_node"), "Failed to set the baudrate!");
        else
            RCLCPP_INFO(rclcpp::get_logger("ax12a_setup_node"), "Succeeded to set the baudrate.");

        setupDynamixel(BROADCAST_ID);

        RCLCPP_INFO(this->get_logger(), "AX-12A setup node running.");
    }

  private:
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;

    void setupDynamixel(uint8_t dxl_id)
    {
        uint8_t dxl_error = 0;
        // Enable Torque of DYNAMIXEL
        int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ax12a_setup_node"), "Failed to enable torque.");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("ax12a_setup_node"), "Succeeded to enable torque.");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ax12aSetupNode>();
    sleep(1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
