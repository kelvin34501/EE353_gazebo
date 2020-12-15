#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <string>
#include <unordered_map>

namespace zz {
enum class Wheel {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
};
}

using zz::Wheel;

namespace gazebo {
/// \brief A plugin to control a Velodyne sensor.
class RobotDrive : public ModelPlugin {
    /// \brief Constructor
    public:
    RobotDrive() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        // Just output a message for now
        std::cerr << "\nThe robot_drive plugin is attach to model[" << _model->GetName() << "]\n";

        // safety check
        if (_model->GetJointCount() == 0) {
            std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
            return;
        }

        // store the model pointer for convenience
        this->model = _model;

// get the first joint. We are making an assumption about the model having one joint that is the rotation joint
#define BASE_INDEX (32)
        auto joints                            = _model->GetJoints();
        this->joint_mapping[Wheel::FrontLeft]  = joints[BASE_INDEX + 0];
        this->joint_mapping[Wheel::RearLeft]   = joints[BASE_INDEX + 1];
        this->joint_mapping[Wheel::FrontRight] = joints[BASE_INDEX + 2];
        this->joint_mapping[Wheel::RearRight]  = joints[BASE_INDEX + 3];

        this->pid_mapping[Wheel::FrontLeft]  = common::PID(0.1, 0, 0);
        this->pid_mapping[Wheel::RearLeft]   = common::PID(0.1, 0, 0);
        this->pid_mapping[Wheel::FrontRight] = common::PID(0.1, 0, 0);
        this->pid_mapping[Wheel::RearRight]  = common::PID(0.1, 0, 0);

        std::cout << "[Joint] FrontLeft: " << this->joint_mapping[Wheel::FrontLeft]->GetScopedName() << '\n';
        std::cout << "[Joint] RearLeft: " << this->joint_mapping[Wheel::RearLeft]->GetScopedName() << '\n';
        std::cout << "[Joint] FrontRight: " << this->joint_mapping[Wheel::FrontRight]->GetScopedName() << '\n';
        std::cout << "[Joint] RearRight: " << this->joint_mapping[Wheel::RearRight]->GetScopedName() << '\n';

        double velocity = 0.0;

        // check  that velocity element exists in sdf, and read value
        if (_sdf->HasElement("velocity")) {
            velocity = _sdf->Get<double>("velocity");
        }

        // apply the p-controller to the joint
        this->model->GetJointController()->SetVelocityPID(this->joint_mapping[Wheel::FrontLeft]->GetScopedName(),
                                                          this->pid_mapping[Wheel::FrontLeft]);
        this->model->GetJointController()->SetVelocityPID(this->joint_mapping[Wheel::RearLeft]->GetScopedName(),
                                                          this->pid_mapping[Wheel::RearLeft]);
        this->model->GetJointController()->SetVelocityPID(this->joint_mapping[Wheel::FrontRight]->GetScopedName(),
                                                          this->pid_mapping[Wheel::FrontRight]);
        this->model->GetJointController()->SetVelocityPID(this->joint_mapping[Wheel::RearRight]->GetScopedName(),
                                                          this->pid_mapping[Wheel::RearRight]);

        this->model->GetJointController()->SetVelocityTarget(this->joint_mapping[Wheel::FrontLeft]->GetScopedName(), 0);
        this->model->GetJointController()->SetVelocityTarget(this->joint_mapping[Wheel::RearLeft]->GetScopedName(), 0);
        this->model->GetJointController()->SetVelocityTarget(this->joint_mapping[Wheel::FrontRight]->GetScopedName(),
                                                             0);
        this->model->GetJointController()->SetVelocityTarget(this->joint_mapping[Wheel::RearRight]->GetScopedName(), 0);

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(this->model->GetWorld()->Name());

        // Create a topic name
        std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

        // Subscribe to the topic, and register a callback
        this->sub = this->node->Subscribe(topicName, &RobotDrive::OnMsg, this);
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private:
    void OnMsg(ConstVector3dPtr& _msg) { this->SetVelocity(_msg->x()); }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public:
    void SetVelocity(const double _vel) {
        // Set the joint's target velocity.
        this->joint_mapping[Wheel::FrontLeft]->SetVelocity(0, _vel);
        this->joint_mapping[Wheel::RearLeft]->SetVelocity(0, _vel);
        this->joint_mapping[Wheel::FrontRight]->SetVelocity(0, _vel);
        this->joint_mapping[Wheel::RearRight]->SetVelocity(0, _vel);
    }

    /// \brief Pointer to the model.
    private:
    physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private:
    std::unordered_map<Wheel, physics::JointPtr> joint_mapping;

    /// \brief PID
    private:
    std::unordered_map<Wheel, common::PID> pid_mapping;

    /// \brief A node used for transport
    private:
    transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private:
    transport::SubscriberPtr sub;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(RobotDrive)
} // namespace gazebo
