#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <thread>
#include <signal.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

//#include "jetbot_pro/pidConfig.h"

#define head1 0xAA
#define head2 0x55
#define sendType_velocity    0x11
#define sendType_pid         0x12
#define sendType_params      0x13

using namespace std;

bool debug = false;
bool need_exit = false;

// Encapsulate boost::asio serial port functionality within a class. This class will handle opening, closing, reading, and writing to the serial port.
class SerialPort {

    public:
    SerialPort(const std::string& port_name, int baud_rate)
        : io_service_(), serial_port_(io_service_) {
        open(port_name, baud_rate);
    }

    ~SerialPort() {
        close();
    }

    void open(const std::string& port_name, int baud_rate) {
        boost::system::error_code ec;
        serial_port_.open(port_name, ec);
        if (ec) {
            throw std::runtime_error("Failed to open serial port: " + ec.message());
        }
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

    void close() {
        if (serial_port_.is_open()) {
            serial_port_.close();
        }
    }

    void write(const uint8_t* data, size_t size) {
        boost::asio::write(serial_port_, boost::asio::buffer(data, size));
    }

    size_t read(uint8_t* buffer, size_t size) {
        return boost::asio::read(serial_port_, boost::asio::buffer(buffer, size));
    }

private:
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;
};

class WaveshareJetbotNode : public rclcpp::Node 
{
    public:
    // A constructor that accepts rclcpp::NodeOptions
    WaveshareJetbotNode(const rclcpp::NodeOptions & options) 
    : Node("waveshare_jetbot_node", options), serial_port_(nullptr) {  
            // Initialize clock
            node_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);    

            // Initialize serial port
            port_name = this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
            baud_rate = this->declare_parameter<int>("baud_rate", 115200);
            serial_port_ = std::make_unique<SerialPort>(port_name, baud_rate);

            //Get robot parameters 
            publish_odom_transform = this->declare_parameter<bool>("publish_odom_transform", true);
            linear_correction = this->declare_parameter<double>("linear_correction", 1.0);
            angular_correction = this->declare_parameter<double>("angular_correction", 1.0);
            SetParams(linear_correction, angular_correction);

            //Get PID parameters 
            kp = this->declare_parameter<int>("kp", 350);
            ki = this->declare_parameter<int>("ki", 120);
            kd = this->declare_parameter<int>("kd", 0);
            SetPID(kp, ki, kd);

            // Get IMU parameters 
            imu_accel_offset_x = this->declare_parameter<double>("imu_accel_offset_x", -0.2741);
            imu_accel_offset_y = this->declare_parameter<double>("imu_accel_offset_y", -0.3470);
            imu_accel_offset_z = this->declare_parameter<double>("imu_accel_offset_z", -0.2192);

            // Create a subscriber for the Twist message
            twist_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&WaveshareJetbotNode::twist_cmd_callback, this, std::placeholders::_1));
            
            // Initialize the transform broadcaster
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            
            //Create publishers for Odometry, IMU, and motor velocities
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
            lvel_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/lvel", 10);
            rvel_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/rvel", 10);
            lset_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/lset", 10);
            rset_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/rset", 10);

            //register pid parameter callback
            on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&WaveshareJetbotNode::pid_parameters_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(),"Node initialized");
            if (debug) {RCLCPP_INFO(this->get_logger(), "need_exit: %s", need_exit ? "true" : "false");}
        }

        int movement_cmd_loop() {
            if (debug) {RCLCPP_INFO(this->get_logger(), "Entering movement_cmd_loop...");}
            rclcpp::Time current_time, last_time;
            if (debug) {RCLCPP_INFO(this->get_logger(), "declared current_time and last_time...");}
            current_time = node_clock->now();  
            last_time = node_clock->now(); 

            RCLCPP_INFO(this->get_logger(),"start movement command loop");
            while (rclcpp::ok() && !need_exit) {
                if (debug) {RCLCPP_INFO(this->get_logger(), "Entering while loop...");}
                current_time = node_clock->now();
                if ((current_time - last_time).seconds() > 0.02) {
                    last_time = current_time;

                    if ((current_time - cmd_time).seconds() > 1) {
                        x = 0.0;
                        y = 0.0;
                        yaw = 0.0;
                    }
                    SetVelocity(x, y, yaw);
                }
                rclcpp::spin_some(shared_from_this());
            }
            // done!
            if (debug) {RCLCPP_INFO(this->get_logger(),"Exiting movement_cmd_loop...");}
            return 0;
        }
        
        int sensors_read_loop()
        {
            enum frameState
            {
                State_Head1, State_Head2, State_Size, State_Data, State_CheckSum, State_Handle
            };
            
            frameState state = State_Head1;
            
            uint8_t frame_size, frame_sum, frame_type;
            uint8_t data[50];
            
            double  imu_list[9];
            double  odom_list[6];
            rclcpp::Time now_time,last_time;

            //Create Publisher message
            sensor_msgs::msg::Imu imu_msgs;
            geometry_msgs::msg::TransformStamped odom_trans;
            geometry_msgs::msg::Quaternion odom_quat;
            nav_msgs::msg::Odometry odom_msgs;
            std_msgs::msg::Int32 lvel_msgs;
            std_msgs::msg::Int32 rvel_msgs;
            std_msgs::msg::Int32 lset_msgs; 
            std_msgs::msg::Int32 rset_msgs;

            RCLCPP_INFO(this->get_logger(),"start sensor read loop");
            
            while(rclcpp::ok() && !need_exit)
            {
                //State machine
                // [head1 head2 size type data checksum ]
                // [0xAA  0x55  0x2D 0x01 ....  0xXX    ]
                switch (state)
                {
                    case State_Head1:             //waiting for frame header 1
                        frame_sum = 0x00;
                        serial_port_->read(&data[0], 1);
                        state = (data[0] == head1 ? State_Head2 : State_Head1);
                        if(state == State_Head1)
                        {
                            //ROS_INFO_STREAM ("recv head1 error : ->"<<(int)data[0]);
                        }
                        break;
                        
                    case State_Head2:             //waiting for frame header 2
                        serial_port_->read(&data[1], 1);
                        state = (data[1] == head2 ? State_Size : State_Head1);
                        if(state == State_Head1)
                        {
                            //ROS_INFO_STREAM ("recv head2 error : ->"<<(int)data[1]);
                        }
                        break;
                        
                    case State_Size:              //waiting for frame Size
                        serial_port_->read(&data[2], 1);
                        frame_size = data[2];
                        state = State_Data;
                        break;
                        
                    case State_Data:              //waiting for frame data
                        serial_port_->read(&data[3], frame_size - 4);
                        frame_type = data[3];
                        state = State_CheckSum;
                        break;
                        
                    case State_CheckSum:         //waiting for frame CheckSum
                        serial_port_->read(&data[frame_size -1], 1);
                        frame_sum = checksum(data,frame_size -1);
                        state = data[frame_size -1] == frame_sum ? State_Handle : State_Head1;
                        if(state == State_Head1)
                        {
                            //ROS_INFO_STREAM ("check sum error! recv is  : ->"<<(int)data[frame_size -1]<<"  calc is "<<frame_sum);
                        }
                        break;
                        
                    case State_Handle:         //processing frame data
                        now_time = node_clock->now();
                        
                        //gyro
                        imu_list[0]=((double)((int16_t)(data[4]*256+data[5]))/32768*2000/180*3.1415);
                        imu_list[1]=((double)((int16_t)(data[6]*256+data[7]))/32768*2000/180*3.1415);
                        imu_list[2]=((double)((int16_t)(data[8]*256+data[9]))/32768*2000/180*3.1415);
                        //Acc 
                        imu_list[3]=((double)((int16_t)(data[10]*256+data[11]))/32768*2*9.8)+imu_accel_offset_x;
                        imu_list[4]=((double)((int16_t)(data[12]*256+data[13]))/32768*2*9.8)+imu_accel_offset_y;
                        imu_list[5]=((double)((int16_t)(data[14]*256+data[15]))/32768*2*9.8)+imu_accel_offset_z;
                        //Angle 
                        imu_list[6]=((double)((int16_t)(data[16]*256+data[17]))/10.0);
                        imu_list[7]=((double)((int16_t)(data[18]*256+data[19]))/10.0);
                        imu_list[8]=((double)((int16_t)(data[20]*256+data[21]))/10.0);
            
                        //publish the IMU message
                        imu_msgs.header.stamp = node_clock->now();
                        imu_msgs.header.frame_id = "base_imu_link";
                        imu_msgs.angular_velocity.x = imu_list[0];
                        imu_msgs.angular_velocity.y = imu_list[1];
                        imu_msgs.angular_velocity.z = imu_list[2];
                        imu_msgs.linear_acceleration.x = imu_list[3];
                        imu_msgs.linear_acceleration.y = imu_list[4];
                        imu_msgs.linear_acceleration.z = imu_list[5];
                        imu_msgs.orientation = tf2::toMsg(createQuaternionFromYaw(imu_list[8]/180*3.1415926));
                        imu_msgs.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 0.7};
                        imu_msgs.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 0.4};
                        // imu_msgs.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6};
                        imu_msgs.linear_acceleration_covariance = {0.2, 0, 0, 0, 0.2, 0, 0, 0, 0.2};
                        imu_pub_->publish(imu_msgs);
                    
                        odom_list[0]=((double)((int16_t)(data[22]*256+data[23]))/1000);
                        odom_list[1]=((double)((int16_t)(data[24]*256+data[25]))/1000);
                        odom_list[2]=((double)((int16_t)(data[26]*256+data[27]))/1000);
                        //dx dy dyaw base_frame
                        odom_list[3]=((double)((int16_t)(data[28]*256+data[29]))/1000);
                        odom_list[4]=((double)((int16_t)(data[30]*256+data[31]))/1000);
                        odom_list[5]=((double)((int16_t)(data[32]*256+data[33]))/1000);
                    
                        //first, we'll publish the transform over tf
                        odom_trans.header.stamp = now_time;
                        odom_trans.header.frame_id = "odom";
                        odom_trans.child_frame_id = "base_link";
            
                        odom_trans.transform.translation.x = odom_list[0];
                        odom_trans.transform.translation.y = odom_list[1];
                        odom_trans.transform.translation.z = 0.0;
                        //we'll need a quaternion created from yaw
                        odom_quat = tf2::toMsg(createQuaternionFromYaw(odom_list[2]));
                        odom_trans.transform.rotation = odom_quat;
            
                        //send the transform
                        if(publish_odom_transform)tf_broadcaster_->sendTransform(odom_trans);
            
                        //next, we'll publish the odometry message over ROS
                        odom_msgs.header.stamp = now_time;
                        odom_msgs.header.frame_id = "odom";
            
                        //set the position
                        odom_msgs.pose.pose.position.x = odom_list[0];
                        odom_msgs.pose.pose.position.y = odom_list[1];
                        odom_msgs.pose.pose.position.z = 0.0;
                        odom_msgs.pose.pose.orientation = odom_quat;
            
                        //set the velocity
                        odom_msgs.child_frame_id = "base_link";
                        odom_msgs.twist.twist.linear.x = odom_list[3]/((now_time-last_time).seconds());
                        odom_msgs.twist.twist.linear.y = odom_list[4]/((now_time-last_time).seconds());
                        odom_msgs.twist.twist.angular.z = odom_list[5]/((now_time-last_time).seconds());
                        //set the covariance to very small value if robot is at rest
                        if (odom_list[3] == 0 && odom_list[4] == 0) {
                            odom_msgs.twist.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                                                        0, 1e-3, 1e-9, 0, 0, 0, 
                                                        0, 0, 1e-9, 0, 0, 0,
                                                        0, 0, 0, 1e-9, 0, 0, 
                                                        0, 0, 0, 0, 1e-9, 0, 
                                                        0, 0, 0, 0, 0, 1e-9 };
                            odom_msgs.pose.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                                                        0, 1e-3, 1e-9, 0, 0, 0, 
                                                        0, 0, 1e-9, 0, 0, 0,
                                                        0, 0, 0, 1e-9, 0, 0, 
                                                        0, 0, 0, 0, 1e-9, 0, 
                                                        0, 0, 0, 0, 0, 1e3 };
                        }
                        //set the covariance to a larger value if robot is moving
                        else{
                            odom_msgs.twist.covariance = { 1e-4, 0, 0, 0, 0, 0, 
                                                        0, 1e-4, 0, 0, 0, 0, 
                                                        0, 0, 1e-9, 0, 0, 0,
                                                        0, 0, 0, 1e-9, 0, 0, 
                                                        0, 0, 0, 0, 1e-9, 0, 
                                                        0, 0, 0, 0, 0, 1e-3 };
                            odom_msgs.pose.covariance = { 0.25, 0, 0, 0, 0, 0, 
                                                        0, 0.25, 0, 0, 0, 0, 
                                                        0, 0, 1e-4, 0, 0, 0,
                                                        0, 0, 0, 1e-4, 0, 0, 
                                                        0, 0, 0, 0, 1e-4, 0, 
                                                        0, 0, 0, 0, 0, 1e-4 };
                        }
                        //publish the odom message
                        odom_pub_->publish(odom_msgs);
                        
                        //publish the motor message
                        lvel_msgs.data = ((int16_t)(data[34]*256+data[35]));
                        rvel_msgs.data = ((int16_t)(data[36]*256+data[37]));
                        lset_msgs.data = ((int16_t)(data[38]*256+data[39]));
                        rset_msgs.data = ((int16_t)(data[40]*256+data[41]));
                        lvel_pub_->publish(lvel_msgs);
                        rvel_pub_->publish(rvel_msgs);
                        lset_pub_->publish(lset_msgs);
                        rset_pub_->publish(rset_msgs);
            
                        last_time = now_time;
            
                        state = State_Head1;
                        break;
                    default:
                        state = State_Head1;
                        break;
                    }
            }
            // done!
            if (debug) {RCLCPP_INFO(this->get_logger(),"Exiting sensor_read_loop...");}
            return 0;
        }

    private:
        std::unique_ptr<SerialPort> serial_port_;
        std::string port_name;
        int baud_rate;
        bool publish_odom_transform;
        int kp;
        int ki;
        int kd;
        double linear_correction;
        double angular_correction;
        double imu_accel_offset_x;
        double imu_accel_offset_y;
        double imu_accel_offset_z;

        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;

        rclcpp::Clock::SharedPtr node_clock;
        rclcpp::Time cmd_time;

        uint8_t checksum(uint8_t* buf, size_t len) 
        {
            uint8_t sum = 0x00;
            for(size_t i=0;i<len;i++)
            {
                sum += *(buf + i);
            }
      
            return sum;
        }
    
        /*PID parameter sending function*/
        void SetPID(int p, int i, int d)
        {
            static uint8_t tmp[11];
            tmp[0] = head1; // Header byte 1
            tmp[1] = head2; // Header byte 2
            tmp[2] = 0x0b;  // Length of the message (11 bytes)
            tmp[3] = sendType_pid; // Command type for sending PID parameters
            tmp[4] = (p >> 8) & 0xff; // High byte of the proportional (P) parameter
            tmp[5] = p & 0xff;        // Low byte of the proportional (P) parameter
            tmp[6] = (i >> 8) & 0xff; // High byte of the integral (I) parameter
            tmp[7] = i & 0xff;        // Low byte of the integral (I) parameter
            tmp[8] = (d >> 8) & 0xff; // High byte of the derivative (D) parameter
            tmp[9] = d & 0xff;        // Low byte of the derivative (D) parameter
            tmp[10] = checksum(tmp, 10); // Calculate checksum for the first 10 bytes
            serial_port_->write(tmp, 11); // Send the 11-byte message via the serial port
        }
    
        /*robot parameter sending function*/
        void SetParams(double linear_correction, double angular_correction) 
        {
            static uint8_t tmp[9];
            tmp[0]  = head1; // Header byte 1
            tmp[1]  = head2; // Header byte 2
            tmp[2]  = 0x09;  // Length of the message (9 bytes)
            tmp[3]  = sendType_params; // Command type for sending parameters
            tmp[4]  = (int16_t)((int16_t)(linear_correction * 1000) >> 8) & 0xff; // High byte of linear correction
            tmp[5]  = (int16_t)(linear_correction * 1000) & 0xff;                // Low byte of linear correction
            tmp[6]  = (int16_t)((int16_t)(angular_correction * 1000) >> 8) & 0xff; // High byte of angular correction
            tmp[7]  = (int16_t)(angular_correction * 1000) & 0xff;                // Low byte of angular correction
            tmp[8]  = checksum(tmp, 8); // Calculate checksum for the first 8 bytes
            serial_port_->write(tmp, 9); // Send the 9-byte message via the serial port
        }
    
        /*robot speed transmission function*/
        void SetVelocity(double x, double y, double yaw)
        {
            static uint8_t tmp[11];
            tmp[0] = head1;
            tmp[1] = head2;
            tmp[2] = 0x0b;
            tmp[3] = sendType_velocity;
            tmp[4] = ((int16_t)(x*1000)>>8) & 0xff;
            tmp[5] = ((int16_t)(x*1000)) & 0xff;
            tmp[6] = ((int16_t)(y*1000)>>8) & 0xff;
            tmp[7] = ((int16_t)(y*1000)) & 0xff;
            tmp[8] = ((int16_t)(yaw*1000)>>8) & 0xff;
            tmp[9] = ((int16_t)(yaw*1000)) & 0xff;
            tmp[10] = checksum(tmp,10);
            serial_port_->write(tmp, 11);
        }
        
        /*cmd_vel subscriver callback function*/
        void twist_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) 
        {
            x = msg->linear.x;
            y = msg->linear.y;
            yaw = msg->angular.z;
            cmd_time = node_clock->now();

            // Log the received linear and angular velocities
            RCLCPP_INFO(this->get_logger(), "Received Twist message:x: %.2f, y: %.2f, yaw: %.2f", x,y, yaw);
        }

        /*pid parameter set callback function*/
        rcl_interfaces::msg::SetParametersResult pid_parameters_callback(std::vector<rclcpp::Parameter> parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
  
          for (const auto &param : parameters) {
            // kp, ki, and kd must be between 0.0 and 2000.0
            if (param.get_name() == "kp") {
              if (param.get_value<int>() < 0 || param.get_value<int>() > 2000) {
                result.successful = false;
                result.reason = "kp must be between 0 and 2000";
                RCLCPP_WARN(this->get_logger(), "Failed to update kp: %s", result.reason.c_str());
                break;
              } else {
                kp = param.get_value<int>();
                SetPID(kp, ki, kd);
                RCLCPP_INFO(this->get_logger(), "Updated kp to: %d", kp);
              }
            } 
            else if (param.get_name() == "ki") {
              if (param.get_value<int>() < 0 || param.get_value<int>() > 2000) {
                result.successful = false;
                result.reason = "ki must be between 0 and 2000";
                RCLCPP_WARN(this->get_logger(), "Failed to update ki: %s", result.reason.c_str());
                break;
              } else {
                ki = param.get_value<int>();
                SetPID(kp, ki, kd);
                RCLCPP_INFO(this->get_logger(), "Updated ki to: %d", ki);
              }
            } 
            else if (param.get_name() == "kd") {
              if (param.get_value<int>() < 0 || param.get_value<int>() > 2000) {
                result.successful = false;
                result.reason = "kd must be between 0 and 2000";
                RCLCPP_WARN(this->get_logger(), "Failed to update kd: %s", result.reason.c_str());
                break;
              } else {
                kd = param.get_value<int>();
                SetPID(kp, ki, kd);
                RCLCPP_INFO(this->get_logger(), "Updated kd to: %d", kd);
              }
            }
          }
          return result;
        };
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        on_set_parameters_callback_handle_;

        tf2::Quaternion createQuaternionFromYaw(double yaw)
        {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return q;
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lvel_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rvel_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lset_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rset_pub_;
    };

void ExitHandler(int sig) {
    (void)sig;
    RCLCPP_INFO(rclcpp::get_logger("ExitHandler"), "Signal received, setting need_exit to true.");
    need_exit = true;
    }

int main(int argc, char * argv[]) {
        rclcpp::init(argc, argv);
        
        // rclcpp::spin(std::make_shared<WaveshareJetbotNode>());
        auto waveshare_jetbot_node = std::make_shared<WaveshareJetbotNode>(rclcpp::NodeOptions());
        signal(SIGINT,ExitHandler);
        
        //Create serial port receiving task
        thread serial_thread([waveshare_jetbot_node]() { waveshare_jetbot_node->sensors_read_loop(); });
        //main thread continue to process movement command
        int ret = waveshare_jetbot_node->movement_cmd_loop();
        
        serial_thread.join();
        rclcpp::shutdown();
        return ret;
    }