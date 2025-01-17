#include "teleop_keyboard_controller.h"

/* Class Definition */
Teleop_Keyboard_Controller::Teleop_Keyboard_Controller(ros::NodeHandle &nh)
{
    // Load in parameters from the ROS parameter server
    std::string steering_topic_name, throttle_topic_name, brakes_topic_name;
    double steering_incr, velocity_incr;
    nh.param("/teleop_keyboard_controller/steering_topic_name", steering_topic_name, std::string("/steering"));
    nh.param("/teleop_keyboard_controller/throttle_topic_name", throttle_topic_name, std::string("/throttle"));
    nh.param("/teleop_keyboard_controller/brakes_topic_name", brakes_topic_name, std::string("/brakes"));
    nh.param("/teleop_keyboard_controller/steering_max", _steering_max, 45.0 * (M_PI/180.0));
    nh.param("/teleop_keyboard_controller/steering_incr", steering_incr, 5.0 * (M_PI/180.0));
    nh.param("/teleop_keyboard_controller/velocity_max", _velocity_max, 20.0);
    nh.param("/teleop_keyboard_controller/velocity_reverse_max", _velocity_reverse_max, 5.0);
    nh.param("/teleop_keyboard_controller/velocity_incr", velocity_incr, 0.25);
    nh.param("/teleop_keyboard_controller/sample_time", _sample_time, 0.1);

    // Report the values of all parameters
    ROS_INFO("steering_topic_name: %s", steering_topic_name.c_str());
    ROS_INFO("throttle_topic_name: %s", throttle_topic_name.c_str());
    ROS_INFO("brakes_topic_name: %s", brakes_topic_name.c_str());
    ROS_INFO("steering_max: %.4f (rad), %.4f (deg)", _steering_max, _steering_max * (180/M_PI));
    ROS_INFO("steering_incr: %.4f (rad), %.4f (deg)", steering_incr, steering_incr * (180/M_PI));
    ROS_INFO("velocity_max: %.4f (m/s)", _velocity_max);
    ROS_INFO("velocity_reverse_max: %.4f (m/s)", _velocity_reverse_max);
    ROS_INFO("velocity_incr: %.4f (m/s)", velocity_incr);
    ROS_INFO("sample_time: %.4f (sec)", _sample_time);
    ROS_INFO("Teleop Keyboard Controller Summary \n%s", _remider_msg);

    // Initialize class members (Non-ROS)
    _steering = 0.0;
    _velocity = 0.0;
    _velocity_reverse_max = (_velocity_reverse_max > _velocity_max) ? _velocity_max : _velocity_reverse_max;
    _brakes = 0.0;
    _key_bindings['w'] = {0.0,              velocity_incr}; // {steering, velocity}
    _key_bindings['s'] = {0.0,              -velocity_incr};
    _key_bindings['a'] = {steering_incr,    0.0,          };
    _key_bindings['d'] = {-steering_incr,   0.0,          };

    // Initialize publishers and subscribers
    _steering_pub   = nh.advertise<std_msgs::Float64>(steering_topic_name, 0, true);
    _throttle_pub   = nh.advertise<std_msgs::Float64>(throttle_topic_name, 0, true);
    _brakes_pub     = nh.advertise<std_msgs::Float64>(brakes_topic_name, 0, true);
}

void Teleop_Keyboard_Controller::update_control_input()
{
    // Get the pressed key
    char key = get_character();

    // Update control inputs if key is valid
    if (_key_bindings.count(key) == 1)
    {   
        _brakes = 0.0;
        vector<double> key_vector = _key_bindings[key];

        _steering += key_vector[0];
        _steering = (_steering > _steering_max) ? _steering_max : 
                    ((_steering < -_steering_max) ? -_steering_max : _steering);
        
        _velocity += key_vector[1];
        if (_velocity >= 0.0)
        {
            _velocity = (_velocity > _velocity_max) ? _velocity_max : _velocity;
        }
        else
        {
            _velocity = (_velocity < -_velocity_reverse_max) ? -_velocity_reverse_max : _velocity;
        } 

        printf("\n\rCurrent Velocity: %.4f \tCurrent Steering Angle: %.4f", _velocity, _steering);
    }

    // Apply brakes if space bar was pressed
    else if (key == ' ')
    {
        _steering = 0.0;
        _velocity = 0.0;
        _brakes = 1.0;

        printf("\n\rBrakes applied");
    }

    // Print exit message if CTRL+C
    else if (key == '\x03')
    {
        printf("\r\nTeleop Keyboard Controller terminating.....bye\r\n");
        ros::shutdown();
    }
}

void Teleop_Keyboard_Controller::publish_msgs() const
{
    // Declare msgs to be published
    std_msgs::Float64 steering_msg;
    std_msgs::Float64 throttle_msg;
    std_msgs::Float64 brakes_msg;

    // Initialize the msgs
    steering_msg.data   = _steering * (180.0/M_PI);
    throttle_msg.data   = _velocity/_velocity_max;
    brakes_msg.data     = _brakes;  

    // Publish all msgs
    _steering_pub.publish(steering_msg);
    _throttle_pub.publish(throttle_msg);
    _brakes_pub.publish(brakes_msg);
}

double Teleop_Keyboard_Controller::get_sample_time() const
{
    return _sample_time;
}

/* End of Class Definition */

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "teleop_keyboard_controller");
    ros::NodeHandle nh;

    Teleop_Keyboard_Controller teleop_keyboard_controller(nh);
    ros::Rate loop_rate(1.0/teleop_keyboard_controller.get_sample_time());

    while (ros::ok())
    {
        teleop_keyboard_controller.update_control_input();
        teleop_keyboard_controller.publish_msgs();
        loop_rate.sleep();
    }

    return 0;
}