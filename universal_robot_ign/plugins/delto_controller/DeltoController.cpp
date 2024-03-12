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
#include "DeltoController.hh"

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

class ignition::gazebo::systems::DeltoControllerPrivate {
   public:
    void getJointPosition(ignition::gazebo::EntityComponentManager &_ecm, double q[12]);
    void setJointVelocity(ignition::gazebo::EntityComponentManager &_ecm, double vel);
    void sendDuty(ignition::gazebo::EntityComponentManager &_ecm, double duty[12]);
    void OnCmd(const ignition::msgs::Int32 &_msg);
    void OnTargetJoint(const ignition::msgs::Float_V &_msg);

   public:
    transport::Node node;
    bool initialized{false};
    //model
    Model model{kNullEntity};
    //gripper joint
    vector<string> jointNames{"F1M1", "F1M2", "F1M3", "F1M4",
                                "F2M1", "F2M2", "F2M3", "F2M4",
                                "F3M1", "F3M2", "F3M3", "F3M4"};

    // vector<int> jointMultipliers{0, 0, 1, 1, 0, 0, -1, -1, 0, 0, 1, 1};
    vector<Entity> jointEntities;
    
    //Gripper Data
    HandControl libHand;
    double q[12] = {0,};
    double pre_q[12] = {0,};
    double q_dot[12] = {0,};
    double kp[12] = {1,1,1,1,1,1,1,1,1,1,1,1};
    double kd[12] = {8,8,8,8,8,8,8,8,8,8,8,8}; 

    double q_d[12] = {0,0,              50.0 * PI / 180.0, 80.0 * PI / 180.0, 
                    -30.0 * PI / 180.0,  0, 50.0 * PI / 180.0, 80.0 * PI / 180.0,
                     30.0 * PI / 180.0,  0, 50.0 * PI / 180.0, 80.0 * PI / 180.0};
    double u[12];
    double duty[12];
    double gf;
    size_t print_counter = 0;
    string gripperName{"gripper"};
    bool isFixed{false};
    double velScale{100.0};
    //grasp object by using detachable joint (contain contact detection)
    Entity gripperBaseLink;
    Entity gripperFingerCollisions[3];
    Entity detachableJointEntity{kNullEntity};
    //gripper cmd
    std::mutex statusMutex;
    GripperStatus gripperStatus = GripperStatus::joint_control;
};

/******************implementation for DeltoController************************/
DeltoController::DeltoController() : dataPtr(std::make_unique<DeltoControllerPrivate>()) 
{
}

void DeltoController::Configure(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> & _sdf,
                                  EntityComponentManager &_ecm,
                                  EventManager & /*_eventMgr*/) { 

    
    ignmsg << "DeltoController configuring" << std::endl;
    this->dataPtr->model = Model(_entity);
    
    if (!this->dataPtr->model.Valid(_ecm))
     {
        ignerr << "DeltoController plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    ignmsg << "DeltoController attached to model [" << this->dataPtr->model.Name(_ecm) << "]" << std::endl;
    // Get params from SDF
    if (_sdf->HasElement("fixed")){
        this->dataPtr->isFixed = _sdf->Get<bool>("fixed");
    }
    if (_sdf->HasElement("vel_scale")){
        this->dataPtr->velScale=_sdf->Get<double>("vel_scale");
    }
    if (_sdf->HasElement("gripper_name")){
        this->dataPtr->gripperName=_sdf->Get<std::string>("gripper_name");
    }
    // Get Gripper Finger Link
    this->dataPtr->gripperBaseLink = this->dataPtr->model.LinkByName(_ecm, "delto_base_link");

    auto linkEntity1 = this->dataPtr->model.LinkByName(_ecm, "F1_04");
    this->dataPtr->gripperFingerCollisions[0] = Link(linkEntity1).CollisionByName(_ecm, "F1_TIP_collision");
    auto linkEntity2 = this->dataPtr->model.LinkByName(_ecm, "F2_04");
    this->dataPtr->gripperFingerCollisions[1] = Link(linkEntity2).CollisionByName(_ecm, "F2_TIP_collision");
    auto linkEntity3 = this->dataPtr->model.LinkByName(_ecm, "F3_04");
    this->dataPtr->gripperFingerCollisions[2] = Link(linkEntity3).CollisionByName(_ecm, "F3_TIP_collision");
   
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
    std::string topic2{"/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->gripperName + "/target_joint"};
    
    this->dataPtr->node.Subscribe(topic1, &DeltoControllerPrivate::OnCmd, this->dataPtr.get());
    this->dataPtr->node.Subscribe(topic2, &DeltoControllerPrivate::OnTargetJoint,this->dataPtr.get());

}

void DeltoController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                  ignition::gazebo::EntityComponentManager &_ecm) {
    //do nothing if it's paused or not initialized.
    if (_info.paused) 
    {
        return;
    }

    //initialize
    if(!this->dataPtr->initialized)
    {
        for (int i = 0; i < 3; i++) 
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

    auto& pr_count =this->dataPtr->print_counter;
    pr_count++;
    if (pr_count > 100)
    {
    std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
    // current Joint Position
    // std::cout << "CurrentJoint Position: " << std::endl;
    // for (int i = 0; i < 12; i++)
    // {
    //     std::cout << this->dataPtr->q[i] << " ";
    // }
    // std::cout << std::endl;

    // //target Joint Position
    // std::cout << "Target Joint Position: " << std::endl;
    // for (int i = 0; i < 12; i++)
    // {
    //     std::cout << this->dataPtr->q_d[i] << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "u: " << std::endl;
    // for (int i = 0; i < 12; i++)
    // {
    //     std::cout << this->dataPtr->u[i] << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "duty: " << std::endl;
    // for (int i = 0; i < 12; i++)
    // {
    //     std::cout << this->dataPtr->duty[i] << " ";
    // }
    // std::cout << std::endl;
    pr_count = 0;
    }

     //FSM

    //self collision detection

    if (gripperStatus == GripperStatus::joint_control) 
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
        this->dataPtr->libHand.jc(this->dataPtr->q,
                     this->dataPtr->q_dot,
                     this->dataPtr->kp,
                     this->dataPtr->kd,
                     this->dataPtr->q_d,
                     this->dataPtr->u);       
    }
    else if(gripperStatus == GripperStatus::grasp1) 
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
        float gf = 10;
        this->dataPtr->libHand.grasp1(gf, this->dataPtr->q, this->dataPtr->u);
        // std::cout << "grasp" << std::endl;      
    }

    else if(gripperStatus == GripperStatus::grasp2)
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
        float gf = 10;
        this->dataPtr->libHand.grasp2(gf, this->dataPtr->q, this->dataPtr->u);
    }
    else if(gripperStatus == GripperStatus::grasp3)
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
        float gf = 10;
        this->dataPtr->libHand.grasp3(gf, this->dataPtr->q, this->dataPtr->u);
    }
    else
    {
        ignerr << "Invalid gripper status" << std::endl;
    }

    this->dataPtr->libHand.u2duty(this->dataPtr->u, this->dataPtr->duty);
    
    for(int i = 0; i < 12; i++)
    {
        this->dataPtr->duty[i] -= (65535/2);
        // this->dataPtr->duty[i] *= 10000.0/(65535/2);
    }
    this->dataPtr->sendDuty(_ecm,
                            this->dataPtr->duty);

    for(int i = 0; i < 12; i++)
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->statusMutex);
        this->dataPtr->q_dot[i] = this->dataPtr->q[i] - this->dataPtr->pre_q[i];
        this->dataPtr->pre_q[i] = this->dataPtr->q[i];
    }

}
void DeltoController::PostUpdate(const ignition::gazebo::UpdateInfo & /*_info*/,
                                   const ignition::gazebo::EntityComponentManager & /*_ecm*/) {
    //do nothing
}

void DeltoControllerPrivate::sendDuty(ignition::gazebo::EntityComponentManager &_ecm, double duty[12]) 
{
    for (size_t i = 0; i < this->jointEntities.size(); i++) 
    {

        auto jointForceCmd = _ecm.Component<components::JointForceCmd>(this->jointEntities[i]);
        jointForceCmd->Data()[0] = duty[i];
    }
}

void DeltoControllerPrivate::getJointPosition(ignition::gazebo::EntityComponentManager &_ecm, double q[12]) 
{
    for (size_t i = 0; i < this->jointEntities.size(); i++) 
    {
        auto jointPosition = _ecm.Component<components::JointPosition>(this->jointEntities[i]);
        q[i] = jointPosition->Data()[0];
    }
}

void DeltoControllerPrivate::OnCmd(const ignition::msgs::Int32 &_msg) {
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

void DeltoControllerPrivate::OnTargetJoint(const ignition::msgs::Float_V &_msg)
{
    std::lock_guard<std::mutex> lock(this->statusMutex);

    // #invalid size check
    if (_msg.data().size() != 12) 
    {
        ignerr << "Invalid target joint position size [" << _msg.data().size() << "]" << std::endl;
        return;
    }

    for (size_t i = 0; i < _msg.data().size(); i++) 
    {
        this->q_d[i] = _msg.data(i);
    }    
}

/******************register*************************************************/
IGNITION_ADD_PLUGIN(DeltoController,
                    ignition::gazebo::System,
                    DeltoController::ISystemConfigure,
                    DeltoController::ISystemPreUpdate,
                    DeltoController::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DeltoController, "ignition::gazebo::systems::DeltoController")