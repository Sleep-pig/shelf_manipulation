#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>

namespace gazebo {
class ShelfGrasp : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr) override {
    robot_ = model;
    fingers_[0] = robot_->GetJoint("panda_finger_joint1");
    fingers_[1] = robot_->GetJoint("panda_finger_joint2");
    contacts_ = robot_->GetWorld()->Physics()->GetContactManager();
    if (!fingers_[0] || !fingers_[1]) {
      gzerr << "Shelf grasp requires both Panda finger joints\n";
      return;
    }
    contacts_->SetNeverDropContacts(true);
    update_ = event::Events::ConnectWorldUpdateEnd([this]() { Update(); });
  }

 private:
  void Update() {
    const double opening = fingers_[0]->Position(0) + fingers_[1]->Position(0);
    if (constraint_) {
      if (opening > 0.074) {
        constraint_->Detach();
        constraint_->Fini();
        constraint_.reset();
      }
      return;
    }
    bool left = false, right = false;
    physics::LinkPtr target;
    for (unsigned i = 0; i < contacts_->GetContactCount(); ++i) {
      const auto *contact = contacts_->GetContact(i);
      auto a = contact->collision1->GetLink();
      auto b = contact->collision2->GetLink();
      if (b->GetModel()->GetName() == "target") std::swap(a, b);
      if (a->GetModel()->GetName() != "target" || b->GetModel() != robot_) continue;
      left |= b->GetName() == "panda_leftfinger";
      right |= b->GetName() == "panda_rightfinger";
      target = a;
    }
    // Require both actual collision pairs in the same physics step.
    if (left && right && opening < 0.075) {
      constraint_ = robot_->GetWorld()->Physics()->CreateJoint("fixed", robot_);
      constraint_->Load(fingers_[0]->GetParent(), target, ignition::math::Pose3d::Zero);
      constraint_->Init();
    }
  }

  physics::ModelPtr robot_;
  physics::JointPtr fingers_[2], constraint_;
  physics::ContactManager *contacts_ = nullptr;
  event::ConnectionPtr update_;
};
GZ_REGISTER_MODEL_PLUGIN(ShelfGrasp)
}  // namespace gazebo
