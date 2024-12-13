#include "lab_auve/uavsim.h"
#include <eigen3/Eigen/Dense>
#include "matplotlibcpp.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// Plot stuff
namespace plt = matplotlibcpp;
std::vector<double> plot_data;
std::vector<double> plot_data2;
std::vector<double> plot_data3;
std::vector<double> plot_data4;

//Usefull variables
double la = 0.17;
double kt = 5.5e-6;
double kd = 3.299e-7;
double drone_mass = 1.0;
double gravity = 9.81;
Matrix4d mixer;
Matrix4d inv_mixer;

float fz, t_phi,t_theta, t_psi;

void ComputeMixerMatrix(){
    mixer << kt, kt, kt, kt,
             -kt*la, kt*la, 0, 0,
             0, 0, -kt*la, kt*la,
             -kd, -kd, kd, kd;
}
void ComputeInverseMixerMatrix(){
    inv_mixer = mixer.inverse();
}
void computeTrajectory(double t, Vector3d& desired_position, Vector3d& desired_velocity) {
    // Trajectory parameters
    double r = 2.0;       // Circle radius (meters)
    double omega = 0.5;   // Angular velocity (rad/s)
    double z0 = 1.0;      // Initial height (meters)
    double vz = 0.1;      // Vertical velocity (m/s)

    // Compute desired position
    desired_position.x() = r * cos(omega * t);
    desired_position.y() = r * sin(omega * t);
    desired_position.z() = z0 + vz * t;

    // Compute desired velocity
    desired_velocity.x() = -r * omega * sin(omega * t);
    desired_velocity.y() = r * omega * cos(omega * t);
    desired_velocity.z() = vz;
}
Vector3d orientationControl(const Quaterniond& desired_orientation, const Quaterniond& current_orientation, const Vector3d current_velocity)
{
    //Diagonal Gain matrix
    Eigen::Vector3d Kd_vals(0.4, 0.4, 0.001);
    Eigen::Vector3d Kp_vals(2.0, 2.0, 0.1);
    Eigen::Matrix3d Kd = Kd_vals.asDiagonal();
    Eigen::Matrix3d Kp = Kp_vals.asDiagonal();
    //Get the error:
    Quaternion attitude_error = current_orientation.inverse()*desired_orientation;
    //Extract real and imaginary parts
    Vector3d q_imaginary = attitude_error.vec();
    double q_real = attitude_error.w();

    //control law:
    Vector3d tau = -Kd*current_velocity + Kp*sgn(q_real)*q_imaginary;
    return tau;
}

Quaterniond positionControl(const Vector3d& desired_position, const Vector3d& desired_velocity,
                             const Vector3d& current_position, const Vector3d& current_velocity,
                             double& scalar_force)
{
    Vector3d pos_error = desired_position - current_position;
    Vector3d vel_error = desired_velocity - current_velocity;
    double m = 1;
    Vector3d g;
    g << 0, 0, -9.81;
    //Diagonal Gain matrix
    Eigen::Vector3d Kd_vals(3.0, 3.0, 3.0);
    Eigen::Vector3d Kp_vals(2.0, 2.0, 2.0);
    Eigen::Matrix3d Kd = Kd_vals.asDiagonal();
    Eigen::Matrix3d Kp = Kp_vals.asDiagonal();

    Vector3d force = Kd*vel_error + Kp*pos_error - m*g;
    scalar_force = force.norm();

    Vector3d z_body = force.normalized();
    Vector3d x_w = Vector3d::UnitX();   //x axis of world frame (equal to x_body)
    Vector3d y_body = z_body.cross(x_w);     //y axis of body frame
    Vector3d x_body = y_body.cross(z_body);  //x axis of body frame

    Matrix3d R;
    R.col(0) = x_body;
    R.col(1) = y_body;
    R.col(2) = z_body;

    Quaterniond desired_orientation(R);

    return desired_orientation;
}

int main()
{
    plt::figure();
    int loop_count=0;

    UAVSim uav = UAVSim();
    double prev_clock = uav.getClock();

    sleep(1); // necessary for proper init before start control

    while (uav.isReadyToGo() and loop_count<5000) {
        if ((uav.getClock() - prev_clock) > 0.005) // Loop at 200Hz
        {
            prev_clock = uav.getClock();
            Vector4d desired_motors;
            // Motor speed should be between 0 and 1466 rpm
            //uav.setMotors(desired_motors);
            loop_count = 0;

            //Test Mixer Matrix
            Vector4d U;
            Vector4d omega_square;
            ComputeMixerMatrix();
            ComputeInverseMixerMatrix();

            double scalar_force = 0;
            Quaterniond current_orientation = uav.getOrientation();
            Vector3d current_angular_velocity = uav.getAngularVelocity();
            Vector3d current_position = uav.getPosition();
            Vector3d current_linear_velocity = uav.getLinearVelocity();

            //Trajectory computation
            Vector3d desired_position;
            Vector3d desired_velocity;
            computeTrajectory(uav.getClock(), desired_position, desired_velocity);

            //Position Controller
            Quaterniond desired_attitude =positionControl(desired_position,desired_velocity,current_position,current_linear_velocity, scalar_force);

            //Attitude controller
                Vector3d tau = orientationControl(desired_attitude, current_orientation, current_angular_velocity);
                //Put calculated torque into thrust vector
                U  << scalar_force,tau;
                //Map it to motor velocities
                omega_square = inv_mixer*U.array().matrix();
                desired_motors = omega_square.array().sqrt();
                //Restrain the motor speeds between 0 (do not change spin direction) and 1466 rpm
                desired_motors = desired_motors.cwiseMax(0).cwiseMin(1466);
                uav.setMotors(desired_motors);

            // A basic example of a scope plot
            // Indeed you need to adapt this to your needs
            // It uses a C++ library based on python matplotlib with similar calls
            plt::clf();
            // You can create as many plots as you want
            plot_data.push_back(desired_position.z());
            plot_data2.push_back(uav.getPosition().z());
            plot_data3.push_back(desired_velocity.x());
            plot_data4.push_back(uav.getLinearVelocity().x());
            plt::plot(plot_data);
            plt::plot(plot_data2);
            plt::plot(plot_data3);
            plt::plot(plot_data4);
            plt::pause(0.0001);
        }
        usleep(1);
        loop_count ++;
    }
    // To plot for you report you can add some code here and use plt::show so the code will wait that you close the figures to stop.
}
