#ifndef TELEOP_KEYBOARD_CONTROLLER
#define TELEOP_KEYBOARD_CONTROLLER

/* C/C++ Header Files */ 
#include <cmath>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

/* ROS Header Files */
#include "ros/ros.h"
#include "std_msgs/Float64.h"

using std::vector;
using std::map;

/* Teleop Keyboard Class */
class Teleop_Keyboard_Controller
{
    public:
    // Constructor and public methods
    Teleop_Keyboard_Controller(ros::NodeHandle &nh);
    void update_control_input();
    void publish_msgs() const;
    double get_sample_time() const;

    private:
    // Private methods

    // Get character (source code from the teleop_twist_keyboard_cpp repository)
    int get_character(void)
    {
        int ch;
        struct termios oldt;
        struct termios newt;

        // Store old settings, and copy to new settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Make required changes and apply the settings
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_iflag |= IGNBRK;
        newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
        newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        tcsetattr(fileno(stdin), TCSANOW, &newt);

        // Get the current character
        ch = getchar();

        // Reapply old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        return ch;
    }

    // Private Members
    
    // Control related
    double _steering;
    double _steering_max;
    double _velocity;
    double _velocity_max;
    double _velocity_reverse_max;
    double _brakes;
    double _sample_time;

    // Keyboard related
    map<char, vector<double>> _key_bindings;
    const char* _remider_msg = R"(
 
    Reading from the keyboard and Publishing to Simulator!
    ---------------------------
    Key Bindings:
        
    w : increase linear velocity by velocity_incr (m/s) while < velocity_max
    s : decrease linear velocity by velocity_incr (m/s) while > -velocity_rear_max
    a : increase steering angle by steering_incr (rad) while < steering_max
    d : decrease steering angle by steering_incr (rad) while > -steering_max
    space bar: lift fully off of throttle and apply full braking power 
    
    CTRL-C to quit
    
    )";

    // ROS related:
    ros::Publisher _steering_pub;
    ros::Publisher _throttle_pub;
    ros::Publisher _brakes_pub;
};


#endif