// this package takes in a cmd_vel message and converts that into speeds for each indivdual wheel in a 4 wheel mechanum drive robot
// meant to be used in conjunction with a differntial drive robot controller
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <math.h>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

double WHEEL_SEPERATION_WIDTH = .203;
double WHEEL_SEPERATION_LENGTH = .18; 
double WHEEL_GEOMETRY = (WHEEL_SEPERATION_LENGTH+WHEEL_SEPERATION_WIDTH)/2;
double WHEEL_RADIUS = .024;

class mechanum_controller_cmd_vel_publisher : public rclcpp::Node
{
    public:
    mechanum_controller_cmd_vel_publisher()
    : Node("mechanum_controller_cmd_vel_publisher")
    {

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",10,std::bind(&mechanum_controller_cmd_vel_publisher::convert_twist_to_mechanum_motor_callback,this,_1));

        publisher_front_left = this->create_publisher<geometry_msgs::msg::Twist>("frontLeft_Joint",10);
        publisher_front_right = this->create_publisher<geometry_msgs::msg::Twist>("frontRight_Joint",10);
        publisher_back_left = this->create_publisher<geometry_msgs::msg::Twist>("backLeft_Joint",10);
        publisher_back_right = this->create_publisher<geometry_msgs::msg::Twist>("backRight_Joint",10);

        //publisher_front_left = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont/frontLeft/cmd_vel_unstamped", 10);
        //publisher_front_right = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont/frontRight/cmd_vel_unstamped", 10);
        //publisher_back_left = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont/backLeft/cmd_vel_unstamped", 10);
        //publisher_back_right = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont/backRight/cmd_vel_unstamped", 10);

    }



    private:
        void convert_twist_to_mechanum_motor_callback(const geometry_msgs::msg::Twist::SharedPtr twist)
            {
                double x = twist->linear.x;
                double y = twist->linear.y;
                double rot = twist->angular.z;

                geometry_msgs::msg::Twist front_left_twist;
                geometry_msgs::msg::Twist front_right_twist;
                geometry_msgs::msg::Twist back_left_twist;
                geometry_msgs::msg::Twist back_right_twist;

                front_left_twist.linear.x = (x -y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS;
                front_right_twist.linear.x = (x + y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS;
                back_left_twist.linear.x = (x + y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS;
                back_right_twist.linear.x = (x - y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS;

                publisher_back_left->publish(back_left_twist);
                publisher_back_right->publish(back_right_twist);
                publisher_front_left->publish(front_left_twist);
                publisher_front_right->publish(front_right_twist);

                
            }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_front_left;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_front_right;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_back_left;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_back_right;
            

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mechanum_controller_cmd_vel_publisher>());
    rclcpp::shutdown();
    return 0;
}

