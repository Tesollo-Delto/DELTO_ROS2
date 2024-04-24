/*******************************************************************************
 *  Copyright (c) Gezp (https://github.com/gezp), All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include "Dg2fController.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/common/Util.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <mutex>

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace std;

enum class GripperStatus {
   joint_control,
   grasp1,
    grasp2,
    grasp3
};

class ignition::gazebo::systems::Dg2fControllerPrivate {
   public:
    void getJointPosition(ignition::gazebo::EntityComponentManager &_ecm, double (&q)[2]);
    void setJointVelocity(ignition::gazebo::EntityComponentManager &_ecm, double vel);
    void sendDuty(ignition::gazebo::EntityComponentManager &_ecm, double (&duty)[2]);
    void OnCmd(const ignition::msgs::Int32 &_msg);
    void OnTargetJoint(const ignition::msgs::Float &_msg);

   public:
    transport::Node node;
    bool initialized{false};
    //model
    Model model{kNullEntity};
    //gripper joint
    vector<string> jointNames{"Distance", "JOINT2"};

    // vector<int> jointMultipliers{0, 0, 1, 1, 0, 0, -1, -1, 0, 0, 1, 1};
    vector<Entity> jointEntities;
    
    //Gripper Data
    // HandControl libHand;
    double q[2] = {0,};
    double pre_q[2] = {0};
    double q_dot[2] = {0};
    double kp[2] = {10,10};
    double kd[2] = {5,5}; 

    double q_d[2] = {0};
    double u[2] = {0};
    double duty[2] = {0};
    // double gf;
    size_t print_counter = 0;
    string gripperName{"gripper"};
    bool isFixed{false};
    double velScale{100.0};
    //grasp object by using detachable joint (contain contact detection)
    Entity gripperBaseLink;
    Entity gripperFingerCollisions[2];
    Entity detachableJointEntity{kNullEntity};
    //gripper cmd
    std::mutex statusMutex;
    GripperStatus gripperStatus = GripperStatus::joint_control;
};

/******************implementation for Dg2fController************************/
Dg2fController::Dg2fController() : dataPtr(std::make_unique<Dg2fControllerPrivate>()) 
{
}

void Dg2fController::Configure(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> & _sdf,
                                  EntityComponentManager &_ecm,
                                  EventManager & /*_eventMgr*/) { 

    
    ignmsg << "Dg2fController configuring" << std::endl;
    this->dataPtr->model = Model(_entity);
    
    if (!this->dataPtr->model.Valid(_ecm))
     {
        ignerr << "Dg2fController plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    ignmsg << "Dg2fController attached to model [" << this->dataPtr->model.Name(_ecm) << "]" << std::endl;
    // Get params from SDF
    if (_sdf->HasElement("fixed")){
        this->dataPtr->isFixed = _sdf->Get<bool>("fixed");
    }
    if (_sdf->HasElement("vel_scale")){
        this->dataPtr->velScale=_sdf->Get<double>("vel_scale");
    }

    if (_sdf->HasElement("kp"))
    {
        this->dataPtr->kp[0]=_sdf->Get<double>("kp");
        this->dataPtr->kp[1]=_sdf->Get<double>("kp");
    }
    if (_sdf->HasElement("kd"))
    {
        this->dataPtr->kd[0]=_sdf->Get<double>("kd");
        this->dataPtr->kd[1]=_sdf->Get<double>("kd");
    }

    if (_sdf->HasElement("gripper_name")){
        this->dataPtr->gripperName=_sdf->Get<std::string>("gripper_name");
    }
    // Get Gripper Finger Link
    this->dataPtr->gripperBaseLink = this->dataPtr->model.LinkByName(_ecm, "delto_base_link");

    auto linkEntity1 = this->dataPtr->model.LinkByName(_ecm, "TIP1");
    this->dataPtr->gripperFingerCollisions[0] = Link(linkEntity1).CollisionByName(_ecm, "TIP1_collision");
    auto linkEntity2 = this->dataPtr->model.LinkByName(_ecm, "TIP2");
    this->dataPtr->gripperFingerCollisions[1] = Link(linkEntity2).CollisionByName(_ecm, "TIP2_collision");

    // Get joint Entity
    for (auto &joint_name : this->dataPtr->jointNames) 
    {
        auto entity = this->dataPtr->model.JointByName(_ecm, joint_name);
        if (entity == kNullEntity) 
        {
            ignerr << "joint with name[" << joint_name << "] not found. " << std::endl;
            return;
        }

        this->dataPtr->jointEntities.push_back(entity);
    }

    // Subscribe to commands
    std::string topic1{"/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->gripperName + "/cmd"};
    std::string topic2{"/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->gripperName + "/target_position"};
    
    this->dataPtr->node.Subscribe(topic1, &Dg2fControllerPrivate::OnCmd, this->dataPtr.get());
    this->dataPtr->node.Subscribe(topic2, &Dg2fControllerPrivate::OnTargetJoint,this->dataPtr.get());

}

void Dg2fController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                  ignition::gazebo::EntityComponentManager &_ecm) {
    //do nothing if it's paused or not initialized.
    if (_info.paused) 
    {
        return;
    }

    //initialize
    if(!this->dataPtr->initialized)
    {
        for (int i = 0; i < 2; i++) 
        {
            auto &colEntity = this->dataPtr->gripperFingerCollisions[i];
            _ecm.CreateComponent(colEntity,components::ContactSensorData());
        }

        for (size_t i = 0; i < this->dataPtr->jointEntities.size(); i++) {
            _ecm.CreateComponent(this->dataPtr->jointEntities[i], components::JointPosition());
            _ecm.CreateComponent(this->dataPtr->jointEntities[i], components::JointForceCmd({0}));
        }

        this->dataPtr->initialized=true;
        return;

    }

    //current state
    GripperStatus gripperStatus;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
        gripperStatus = this->dataPtr->gripperStatus;
    }

    this->dataPtr->getJointPosition(_ecm, this->dataPtr->q);

    //cout<< "joint_entity: " << this->dataPtr->jointEntities.size() << endl;
    
    auto& pr_count =this->dataPtr->print_counter;
    pr_count++;


    if (pr_count > 100)
    {
        // debug cout at 1/100 * control rate

        // std::cout << "duty: " << this->dataPtr->duty[0] << " " << this->dataPtr->duty[1] << std::endl;


        pr_count = 0;
    }

     //FSM
    
    // this->dataPtr->duty = this->dataPtr->kp * (this->dataPtr->q_d - this->dataPtr->q)
    //                      + this->dataPtr->kd * (0 - this->dataPtr->q_dot);
    // this->dataPtr->duty *= 10;

    for (size_t i = 0; i < 2; i++) 
    {
        this->dataPtr->duty[i] = this->dataPtr->kp[i] * (this->dataPtr->q_d[i] - this->dataPtr->q[i])
                                + this->dataPtr->kd[i] * (0 - this->dataPtr->q_dot[i]);

        this->dataPtr->duty[i] *= 10;
    }

    this->dataPtr->sendDuty(_ecm, this->dataPtr->duty);


    for (size_t i = 0; i < 2; i++) 
    {
        this->dataPtr->q_dot[i] = this->dataPtr->q[i] - this->dataPtr->pre_q[i];
        this->dataPtr->pre_q[i] = this->dataPtr->q[i];
    }
}

void Dg2fController::PostUpdate(const ignition::gazebo::UpdateInfo & /*_info*/,
                                   const ignition::gazebo::EntityComponentManager & /*_ecm*/) 
{
    //do nothing
}

void Dg2fControllerPrivate::sendDuty(ignition::gazebo::EntityComponentManager &_ecm, double (&duty)[2])
{

        auto jointForceCmd = _ecm.Component<components::JointForceCmd>(this->jointEntities[0]);
        jointForceCmd->Data()[0] = duty[0];
        jointForceCmd = _ecm.Component<components::JointForceCmd>(this->jointEntities[1]);
        jointForceCmd->Data()[0] = duty[1];
    
}

void Dg2fControllerPrivate::getJointPosition(ignition::gazebo::EntityComponentManager &_ecm, double (&q)[2]) 
{
    for (size_t i = 0; i < this->jointEntities.size(); i++) 
    {
        auto jointPosition = _ecm.Component<components::JointPosition>(this->jointEntities[i]);
        q[i] = jointPosition->Data()[0];

    }
}

void Dg2fControllerPrivate::OnCmd(const ignition::msgs::Int32 &_msg) {
    std::lock_guard<std::mutex> lock(this->statusMutex);
    //is grasp?
    if (_msg.data() == 0) 
    {
        this->gripperStatus = GripperStatus::joint_control; 
    } 
    else if(_msg.data() == 1)
    {
        this->gripperStatus = GripperStatus::grasp1;
    }
    else if(_msg.data() == 2)
    {
        this->gripperStatus = GripperStatus::grasp2;
    }
    else if(_msg.data() == 3)
    {
        this->gripperStatus = GripperStatus::grasp3;
    }
    else
    {
        ignerr << "Invalid command [" << _msg.data() << "]" << std::endl;
    }
}

void Dg2fControllerPrivate::OnTargetJoint(const ignition::msgs::Float &_msg)
{
    std::lock_guard<std::mutex> lock(this->statusMutex);

    this->q_d[0] = _msg.data()/2;
    this->q_d[1] = _msg.data()/2;
     
}

/******************register*************************************************/
IGNITION_ADD_PLUGIN(Dg2fController,
                    ignition::gazebo::System,
                    Dg2fController::ISystemConfigure,
                    Dg2fController::ISystemPreUpdate,
                    Dg2fController::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Dg2fController, "ignition::gazebo::systems::Dg2fController")