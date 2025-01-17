#ifndef OPEN_LOOP_CONTROLLER
#define OPEN_LOOP_CONTROLLER

/* C/C++ Header Files */
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Dense>

/* ROS Header Files */
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"

using std::vector;
using Eigen::Vector3d;

/* Open Loop Controller Class */
class Open_Loop_Controller
{
    public:
    // Constructor and public methods
    Open_Loop_Controller(ros::NodeHandle &nh);
    void callback_trajectory(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void update_control_input();
    void update_control_input_new();
    void publish_msgs() const;
    double get_sample_time() const;

    private:
    // Private methods
    void generate_time_steps(uint32_t traj_length)
    {
        // Generate time steps assuming constant acceleration between points
        float delta_x, delta_y, euclid_dist, avg_vel;
        _traj_time[0] = 0.0f;     
        for (uint32_t i = 1; i < traj_length; i++)
        {
            delta_x = _traj_x[i] - _traj_x[i-1];
            delta_y = _traj_y[i] - _traj_y[i-1];
            euclid_dist = sqrtf(delta_x * delta_x + delta_y * delta_y);
            avg_vel = (_traj_vel[i] + _traj_vel[i-1])/2.0f;
            _traj_time[i] = (fabs(avg_vel) > 1e-4f) ? _traj_time[i-1] + _traj_time_scale_factor * euclid_dist/avg_vel : _traj_time[i-1];
        }
        ROS_INFO("Trajectory Time: %.4f", _traj_time[traj_length-1]);
    }

    bool find_next_point(const float elapsed_time)
    {
        // Brute force search
        uint32_t traj_length = _traj_x.size();
        if (__builtin_expect(traj_length != 0, true))
        {
            for (uint32_t i = _traj_prev_index; i < traj_length; i++)
            {
                if (_traj_time[i] > elapsed_time)
                {
                    _traj_prev_index = i-1;
                    return true;
                }
            }
            _traj_prev_index = traj_length;
            return false;
        }
        else
        {
            return false;
        }
    }

    void update_pose()
    {
        double beta = atan2(_rear_to_cg * tan(_steering), _wheelbase);

        // Define a lambda function for computing the motion model
        auto motion_model = [this, &beta](const Vector3d &pose) -> Vector3d
        {
            return Vector3d(_velocity * cos(beta + pose(2)), 
                            _velocity * sin(beta + pose(2)), 
                            _velocity * tan(_steering) * cos(beta)/_wheelbase);
        };

        // Update pose using motion model (RK4) and inputs
        Vector3d k1 = motion_model(_pose_current);
        Vector3d k2 = motion_model(_pose_current + (_sample_time/2.0) * k1);
        Vector3d k3 = motion_model(_pose_current + (_sample_time/2.0) * k2);
        Vector3d k4 = motion_model(_pose_current + _sample_time * k3);

        _pose_current += _sample_time * ((k1 + 2 * k2 + 2 * k3 + k4)/6.0); 
    }

    void update_nearest_neighbor(double &curvature, double &velocity) const
    {
        uint32_t traj_length = _traj_x.size();
        if (__builtin_expect(traj_length != 0, true))
        {
            float min_dist = std::numeric_limits<float>::infinity();
            float min_dist_lookahead = std::numeric_limits<float>::infinity();
            uint32_t nearest_neighbor, nearest_neighbor_lookahead;
            float delta_x, delta_y, euclid_dist, euclid_dist_lookahead;
            for (uint32_t i = 0; i < traj_length; i++)
            {
                delta_x = _traj_x[i] - _pose_current(0);
                delta_y = _traj_y[i] - _pose_current(1); 
                euclid_dist = sqrtf(delta_x * delta_x + delta_y * delta_y);
                euclid_dist_lookahead = fabs(euclid_dist - _lookahead);

                if (euclid_dist < min_dist)
                {
                    min_dist = euclid_dist;
                    nearest_neighbor = i;
                }

                if ((euclid_dist_lookahead < min_dist_lookahead) && (abs(atan2(delta_y, delta_x)) < M_PI_2))
                {
                    min_dist_lookahead = euclid_dist_lookahead;
                    nearest_neighbor_lookahead = i;
                }
            }

            curvature = static_cast<double>(_traj_kappa[nearest_neighbor_lookahead]);
            velocity = static_cast<double>(_traj_vel[nearest_neighbor]);
        }
        else
        {
            curvature = 0.0;
            velocity = 0.0;
        }
    }

    // Private members

    // Trajectory related
    vector<float> _traj_x;
    vector<float> _traj_y;
    vector<float> _traj_kappa;
    vector<float> _traj_vel;
    vector<float> _traj_time;
    float _traj_time_scale_factor;
    ros::Time _traj_start_time;
    int32_t _traj_prev_index;

    // Pose (2D) related
    Vector3d _pose_current;

    // Control related
    double _steering;
    double _velocity;
    double _velocity_max;
    double _wheelbase;
    double _rear_to_cg;
    double _sample_time;
    float _lookahead;

    // ROS publishers and subscribers
    ros::Publisher _steering_pub;
    ros::Publisher _throttle_pub;
    ros::Publisher _pose_pub;
    ros::Subscriber _traj_sub;
};

#endif