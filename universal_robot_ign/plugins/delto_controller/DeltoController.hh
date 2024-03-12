/*******************************************************************************
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/

#ifndef IGNITION_GAZEBO_SYSTEMS_DELTO_CONTROLLER_HH
#define IGNITION_GAZEBO_SYSTEMS_DELTO_CONTROLLER_HH

#include <memory>
#include <ignition/gazebo/System.hh>
#include "HandControl.h"

namespace ignition
{
    namespace gazebo
    {
        namespace systems
        {
            class DeltoControllerPrivate;

            class IGNITION_GAZEBO_VISIBLE DeltoController
                : public ignition::gazebo::System,
                  public ISystemConfigure,
                  public ISystemPreUpdate,
                  public ISystemPostUpdate
            {
            public:
                DeltoController();
                ~DeltoController() override = default;

            public:
                void Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr) override;
                void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                               ignition::gazebo::EntityComponentManager &_ecm) override;
                void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                const ignition::gazebo::EntityComponentManager &_ecm) override;

            private:
                std::unique_ptr<DeltoControllerPrivate> dataPtr;
            };
        } // namespace systems
    }     // namespace gazebo
} // namespace ignition

#endif