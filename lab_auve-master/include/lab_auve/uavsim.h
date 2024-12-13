#ifndef UAVSIM_H
#define UAVSIM_H

#include <eigen3/Eigen/Geometry>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

using namespace Eigen;

class UAVSim
{
public:
    UAVSim();
    ~UAVSim();

    const Quaterniond &getOrientation() const;
    const Vector3d &getPosition() const;
    const double &getClock() const;
    bool isReadyToGo() const;
    void setMotors(const Vector4d &motor_inputs);

    const Vector3d &getAngularVelocity() const;

    const Vector3d &getLinearVelocity() const;

private:
    void poseCallback(const ignition::msgs::Pose_V &poses);
    void clockCallback(const ignition::msgs::Clock &m_clock);

    bool ready_to_go;
    bool velocity_init;

    double clock;
    Quaterniond orientation;
    Vector3d position;
    Quaterniond last_orientation;
    double last_stamp;
    Vector3d last_position;
    Vector3d angular_velocity;
    Vector3d linear_velocity;

    ignition::transport::Node node;
    ignition::transport::Node::Publisher actuators_pub;
};

#endif // UAVSIM_H
