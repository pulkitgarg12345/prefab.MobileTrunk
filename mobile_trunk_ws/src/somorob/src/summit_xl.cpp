#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class TeleopTwistTest : public rclcpp::Node 
{
  public:
    TeleopTwistTest() : Node("teleop_twist_test")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/summit_xl/robotnik_base_control/cmd_vel", 100);
        timer_ = this->create_wall_timer(10ms, std::bind(&TeleopTwistTest::timer_callback, this));
        
    }

  private:
    void timer_callback()
    {
        time_now_ = this->get_clock()->now();

        auto dt_duration = time_now_ - time_s_;
        double dt = dt_duration.seconds();


        if(dt > 0.0 && dt < 10.0)
        {
            RCLCPP_INFO(this->get_logger(), "etape 1 = %f", dt);
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.3; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0;
            publisher_->publish(twist);
        }

        if(dt > 10.0 && dt < 20.0)
        {
            RCLCPP_INFO(this->get_logger(), "etape 2 = %f", dt);
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = M_PI / 20.0;
            publisher_->publish(twist);
        }

         if(dt > 20.0 && dt < 30.0)
        {
            RCLCPP_INFO(this->get_logger(), "etape 3 = %f", dt);
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.3; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.;
            publisher_->publish(twist);
        }

          if(dt > 30.0 && dt < 40.0)
        {
            RCLCPP_INFO(this->get_logger(), "etape 4 = %f", dt);
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.; twist.linear.y = 0.0; twist.linear.z = 0.0; 
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = M_PI / 20.0;
            publisher_->publish(twist);
        }

          if(dt > 40.0 && dt < 50.0)
        {
            RCLCPP_INFO(this->get_logger(), "etape 5 = %f", dt);
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.3; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.;
            publisher_->publish(twist);
        }


          if(dt > 50.0 && dt < 60.0)
        {
            RCLCPP_INFO(this->get_logger(), "etape 6 = %f", dt);
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.; twist.linear.y = 0.0; twist.linear.z = 0.0; 
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z =  M_PI / 20.0;
            publisher_->publish(twist);
        }

          if(dt > 60.0 && dt < 70.0)
        {
            RCLCPP_INFO(this->get_logger(), "etape 7 = %f", dt);
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.3; twist.linear.y = 0.0; twist.linear.z = 0.0; 
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z =0.0;
            publisher_->publish(twist);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time time_now_;
     rclcpp::Time time_s_ = this->get_clock()->now();
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopTwistTest>());
    rclcpp::shutdown();
    rclcpp::shutdown(); // Ferme le terminal ROS

    return 0;
}