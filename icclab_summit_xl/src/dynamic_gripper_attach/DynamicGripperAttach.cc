#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/empty.pb.h>
#include <sdf/Element.hh>
#include <mutex>

namespace icclab {

class DynamicGripperAttach
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    parentModelName_ = sdf->Get<std::string>("parent_model", "summit").first;
    parentLinkName_  = sdf->Get<std::string>("parent_link",  "arm_wrist_3_link").first;
    std::string attachTopic = sdf->Get<std::string>("attach_topic", "/gripper/attach").first;
    std::string detachTopic = sdf->Get<std::string>("detach_topic", "/gripper/detach").first;

    node_.Subscribe(attachTopic, &DynamicGripperAttach::OnAttach, this);
    node_.Subscribe(detachTopic, &DynamicGripperAttach::OnDetach, this);

    gzmsg << "[DynamicGripperAttach] parent=" << parentModelName_
          << "/" << parentLinkName_
          << "  attach=" << attachTopic
          << "  detach=" << detachTopic << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &ecm) override
  {
    // Resolve parent link lazily — robot model may not exist at Configure time.
    if (parentLinkEntity_ == gz::sim::kNullEntity) {
      parentLinkEntity_ = FindLink(ecm, parentModelName_, parentLinkName_);
      if (parentLinkEntity_ != gz::sim::kNullEntity) {
        gzmsg << "[DynamicGripperAttach] parent link resolved: "
              << parentModelName_ << "/" << parentLinkName_ << "\n";
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (pendingAttach_) {
      DoAttach(ecm);
      pendingAttach_ = false;
    }
    if (pendingDetach_) {
      DoDetach(ecm);
      pendingDetach_ = false;
    }
  }

private:
  void OnAttach(const gz::msgs::StringMsg &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pendingChildModel_ = msg.data();
    pendingAttach_     = true;
    pendingDetach_     = false;  // cancel any queued detach
  }

  void OnDetach(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pendingDetach_ = true;
    pendingAttach_ = false;  // cancel any queued attach
  }

  void DoAttach(gz::sim::EntityComponentManager &ecm)
  {
    // Detach existing joint first (re-attach to a different object)
    DoDetach(ecm);

    if (parentLinkEntity_ == gz::sim::kNullEntity) {
      gzerr << "[DynamicGripperAttach] parent link not yet available: "
            << parentModelName_ << "/" << parentLinkName_ << "\n";
      return;
    }

    gz::sim::Entity childLink = FindRootLink(ecm, pendingChildModel_);
    if (childLink == gz::sim::kNullEntity) {
      gzerr << "[DynamicGripperAttach] child model not found: "
            << pendingChildModel_ << "\n";
      return;
    }

    jointEntity_ = ecm.CreateEntity();
    gz::sim::components::DetachableJointInfo info;
    info.parentLink = parentLinkEntity_;
    info.childLink  = childLink;
    info.jointType  = "fixed";
    ecm.CreateComponent(jointEntity_, gz::sim::components::DetachableJoint(info));

    gzmsg << "[DynamicGripperAttach] attached " << pendingChildModel_ << "\n";
  }

  void DoDetach(gz::sim::EntityComponentManager &ecm)
  {
    if (jointEntity_ != gz::sim::kNullEntity) {
      ecm.RequestRemoveEntity(jointEntity_);
      jointEntity_ = gz::sim::kNullEntity;
      gzmsg << "[DynamicGripperAttach] detached\n";
    }
  }

  // Find a named link inside a named model
  gz::sim::Entity FindLink(gz::sim::EntityComponentManager &ecm,
                            const std::string &modelName,
                            const std::string &linkName)
  {
    gz::sim::Entity result = gz::sim::kNullEntity;
    ecm.Each<gz::sim::components::Link,
             gz::sim::components::Name,
             gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &e,
          const gz::sim::components::Link *,
          const gz::sim::components::Name *name,
          const gz::sim::components::ParentEntity *parent) -> bool
      {
        if (name->Data() != linkName) return true;
        auto *parentName = ecm.Component<gz::sim::components::Name>(parent->Data());
        if (parentName && parentName->Data() == modelName) {
          result = e;
          return false;
        }
        return true;
      });
    return result;
  }

  // Walk up the parent chain and return true if modelEntity is an ancestor of e.
  bool IsDescendantOf(gz::sim::EntityComponentManager &ecm,
                      gz::sim::Entity e,
                      gz::sim::Entity modelEntity)
  {
    auto *p = ecm.Component<gz::sim::components::ParentEntity>(e);
    while (p) {
      if (p->Data() == modelEntity) return true;
      p = ecm.Component<gz::sim::components::ParentEntity>(p->Data());
    }
    return false;
  }

  // Find the first link anywhere inside the named model (handles nested models).
  gz::sim::Entity FindRootLink(gz::sim::EntityComponentManager &ecm,
                                const std::string &modelName)
  {
    gz::sim::Entity modelEntity = gz::sim::kNullEntity;
    ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
      [&](const gz::sim::Entity &e,
          const gz::sim::components::Model *,
          const gz::sim::components::Name *name) -> bool
      {
        if (name->Data() == modelName) { modelEntity = e; return false; }
        return true;
      });

    if (modelEntity == gz::sim::kNullEntity) return gz::sim::kNullEntity;

    gz::sim::Entity firstLink = gz::sim::kNullEntity;
    ecm.Each<gz::sim::components::Link>(
      [&](const gz::sim::Entity &e,
          const gz::sim::components::Link *) -> bool
      {
        if (IsDescendantOf(ecm, e, modelEntity)) { firstLink = e; return false; }
        return true;
      });
    return firstLink;
  }

  gz::transport::Node node_;
  std::mutex mutex_;
  std::string parentModelName_, parentLinkName_, pendingChildModel_;
  bool pendingAttach_{false}, pendingDetach_{false};
  gz::sim::Entity parentLinkEntity_{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity_{gz::sim::kNullEntity};
};

} // namespace icclab

GZ_ADD_PLUGIN(icclab::DynamicGripperAttach,
              gz::sim::System,
              icclab::DynamicGripperAttach::ISystemConfigure,
              icclab::DynamicGripperAttach::ISystemPreUpdate)
