#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

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
        this->joint = _model->GetJoints()[0];

        double velocity = 0.0;

        // check  that velocity element exists in sdf, and read value
        if (_sdf->HasElement("velocity")) {
            velocity = _sdf->Get<double>("velocity");
        }

        // apply the p-controller to the joint
        this->model->GetJoint(this->joint->GetScopedName())->SetVelocity(0, velocity);

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
    void SetVelocity(const double& _vel) {
        // Set the joint's target velocity.
        this->model->GetJoint(this->joint->GetScopedName())->SetVelocity(0, _vel);
    }

    /// \brief Pointer to the model.
    private:
    physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private:
    physics::JointPtr joint;

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
