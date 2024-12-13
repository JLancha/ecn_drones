#include "lab_auve/uavsim.h"
#include <unistd.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

UAVSim::UAVSim():
    ready_to_go(false),
    velocity_init(false)
{
    std::string motor_speed_topic = "/crazy2fly_0/command/motor_speed";
    std::string pose_topic = "/world/default/pose/info";
    std::string clock_topic = "/clock";
    if (!node.Subscribe(pose_topic, &UAVSim::poseCallback, this)) {
        std::cout << "Error: Pose topic not found, is Gazebo running?" << std::endl;
        return;
    }
    if (!node.Subscribe(clock_topic, &UAVSim::clockCallback, this)) {
        std::cout << "Error: Clock topic not found." << std::endl;
        return;
    }
    actuators_pub = node.Advertise<ignition::msgs::Actuators>(motor_speed_topic);
    if (!actuators_pub.Valid())
    {
        std::cout << "Error: Actuator topic not initialized." << std::endl;
        return;
    }
    while(clock==0)
        usleep(10);
    linear_velocity << 0,0,0;
    angular_velocity << 0,0,0;
    ready_to_go=true;
}

UAVSim::~UAVSim()
{
    for (auto &sub_topic : node.SubscribedTopics()) {
        node.Unsubscribe(sub_topic);
    }
}

bool UAVSim::isReadyToGo() const
{
    return ready_to_go;
}

void UAVSim::setMotors(const Vector4d& motor_inputs)
{
    ignition::msgs::Actuators rotor_message;
    rotor_message.mutable_velocity()->Resize(4, 0);
    for (int i = 0; i < 4; i++)
        rotor_message.set_velocity(i, motor_inputs[i]);
    actuators_pub.Publish(rotor_message);
}

const Vector3d &UAVSim::getPosition() const
{
    return position;
}

const Quaterniond &UAVSim::getOrientation() const
{
    return orientation;
}

void UAVSim::poseCallback(const ignition::msgs::Pose_V &poses)
{
    double stamp = poses.header().stamp().sec() + poses.header().stamp().nsec()/1e9;
    for (auto const &pose: poses.pose())
    {
        if (pose.name()=="crazy2fly_0")
        {
            position << pose.position().x(),
                        pose.position().y(),
                        pose.position().z();

            orientation = Quaterniond(
                           pose.orientation().w(),
                           pose.orientation().x(),
                           pose.orientation().y(),
                           pose.orientation().z()
                                      );
            orientation = orientation.normalized();
        }
    }

    double dt = stamp - last_stamp;
    if (velocity_init)
        linear_velocity = (position - last_position)/dt;
    // handles the quaternion singularity
    Quaterniond::Coefficients diff_quat;
    if ((orientation.coeffs() - last_orientation.coeffs()).norm() < (orientation.coeffs() + last_orientation.coeffs()).norm())
        diff_quat = orientation.coeffs() - last_orientation.coeffs();
    else
        diff_quat = orientation.coeffs() + last_orientation.coeffs();
    // w_local = 2  qD * q^(-1)
    if (velocity_init)
        angular_velocity = 2 * (orientation.inverse() * Quaterniond(diff_quat / dt)).vec();

    last_position = position;
    last_orientation=orientation;
    last_stamp = stamp;
    velocity_init = true;
}

void UAVSim::clockCallback(const ignition::msgs::Clock &m_clock)
{
    clock = m_clock.sim().sec() + m_clock.sim().nsec()/1e9;
}

const Vector3d &UAVSim::getLinearVelocity() const
{
    return linear_velocity;
}

const Vector3d &UAVSim::getAngularVelocity() const
{
    return angular_velocity;
}

const double &UAVSim::getClock() const
{
    return clock;
}
