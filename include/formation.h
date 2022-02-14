/*
 * formation.h
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2022 Matthew (matthewoots at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */

/*
* Implementation of https://www.researchgate.net/publication/315592991_Leader-Follower_Formation_Control_for_Quadrotors 
* by Falin Wu and Liang Yuan from Beihang University
*/

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;

#define pi 3.141592

class simple_formation_controller
{
private:
    
    double previous_yaw_e;
    double previous_integral_yaw_e;
    Vector3d previous_pos_e;
    Vector3d previous_integral_pos_e;
    double p_gain, i_gain, d_gain, dt;

    /**
     * Wrap integer value to stay in range [low, high)
     *
     * @param x input possibly outside of the range
     * @param low lower limit of the allowed range
     * @param high upper limit of the allowed range
     * @return wrapped value inside the range
     */
    double wrap(double x, double low, double high) 
    {
        double range = high - low;
        if (x < low) 
            x += range * ((low - x) / range + 1);

        return low + fmod((x - low), range);
    }

    /**
     * Wrap value in range [-pi, pi)
     */
    double wrap_pi(double x)
    {
        return wrap(x, -pi, pi);
    }

public:

    void set_gains_dt(double p, double i, double d, double t)
    {
        p_gain = p; i_gain = i; d_gain = d; t = dt;
    }

    Vector3d position_controller(Vector3d global_pos, Vector3d leader_pos, 
        Vector3d pattern_offset, float yaw, float desired_yaw)
    {
        Vector3d pid_pos = Vector3d::Zero();
        pid_pos = controller_pid(global_pos, leader_pos, 
        pattern_offset, yaw, desired_yaw);

        /*
        * Since PID pose is relative to the the leader pose,
        * we should convert back from leader pose to global
        */
        // pid_pos(0) * sin(desired_yaw) + ;
    }

    Vector3d controller_pid(Vector3d global_pos, Vector3d leader_pos, 
        Vector3d pattern_offset, float yaw, float desired_yaw)
    {
        /* Using ENU */
        double psi_L = wrap_pi(desired_yaw - yaw);
        double X_L = leader_pos(0); double X_F = global_pos(0);
        double Y_L = leader_pos(1); double Y_F = global_pos(1);
        double Z_L = leader_pos(2); double Z_F = global_pos(2);
        double lambda_x = lx(X_L, X_F, Y_L, Y_F, psi_L);
        double lambda_y = ly(X_L, X_F, Y_L, Y_F, psi_L);

        double lambda_desired_x = lx(X_L, pattern_offset(0), Y_L, pattern_offset(1), 0);
        double lambda_desired_y = ly(X_L, pattern_offset(0), Y_L, pattern_offset(1), 0);
    
        double e_x = lambda_desired_x - lambda_x;
        float d_e_x = (e_x - previous_pos_e(0)) / dt; // Derivative
        float i_e_x = previous_integral_pos_e(0) + e_x * dt; // Integral

        double e_y = lambda_desired_y - lambda_y;
        float d_e_y = (e_y - previous_pos_e(1)) / dt; // Derivative
        float i_e_y = previous_integral_pos_e(1) + e_y * dt; // Integral

        Vector3d pid_pos = Vector3d::Zero();

        // In the leader's body frame
        pid_pos.x() = p_gain * e_x + 
            i_gain * i_e_x +
            d_gain * d_e_x;
        
        pid_pos.y() = p_gain * e_y + 
            i_gain * i_e_y +
            d_gain * d_e_y;

        return pid_pos;
    }

    double lx(double X_L, double X_F, double Y_L, double Y_F, double psi_L)
    {
        return -(X_L - X_F) * cos(psi_L) - (Y_L - Y_F) * sin(psi_L);
    }

    double ly(double X_L, double X_F, double Y_L, double Y_F, double psi_L)
    {
        return (X_L - X_F) * sin(psi_L) - (Y_L - Y_F) * cos(psi_L);
    }

};
