#include "rclcpp/rclcpp.hpp"
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "unitree_actuator_sdk_ros2/msg/motor_cmd.hpp"
#include "unitree_actuator_sdk_ros2/msg/motor_data.hpp"
#include <memory>

class Motor : public rclcpp::Node
{
    rclcpp::Publisher<unitree_actuator_sdk_ros2::msg::MotorData>::SharedPtr publisher;
    rclcpp::Subscription<unitree_actuator_sdk_ros2::msg::MotorCmd>::SharedPtr subscription;
    SerialPort* serial;
    public: Motor(): Node("motor"){
        serial = new SerialPort("/dev/ttyS7"); // edit here based on your port
        publisher = this->create_publisher<unitree_actuator_sdk_ros2::msg::MotorData>("motor_data",10);
        subscription = this->create_subscription<unitree_actuator_sdk_ros2::msg::MotorCmd>(
            "motor_cmd", 10, std::bind(&Motor::motor_cmd_callback, this, std::placeholders::_1));
    }
    private:
        void motor_cmd_callback(const unitree_actuator_sdk_ros2::msg::MotorCmd::SharedPtr msg)const{
            unitree_actuator_sdk_ros2::msg::MotorData motorData = unitree_actuator_sdk_ros2::msg::MotorData();
            std::shared_ptr<MotorCmd> cmd = std::make_shared<MotorCmd>();
            std::shared_ptr<MotorData> data = std::make_shared<MotorData>();
            cmd->motorType = MotorType::GO_M8010_6;
            data->motorType = MotorType::GO_M8010_6;
            cmd->id = msg->id;
            cmd->mode = msg->mode;
            cmd->tau = msg->tau;
            cmd->kp = msg->kp;
            cmd->kd = msg->kd;
            cmd->q = msg->pos;
            cmd->dq = msg->vel;
            serial->sendRecv(cmd.get(),data.get());
            motorData.id = data->motor_id;
            motorData.mode = data->mode;
            motorData.tau = data->tau;
            motorData.vel = data->dq;
            motorData.pos = data->q;
            motorData.temp = data->temp;
            motorData.error = data->merror;
            motorData.force = data->footForce;
            publisher->publish(motorData);
        }
};

int main(int argc,char* argv[]){
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Motor>());
	rclcpp::shutdown();
    return 0;
}