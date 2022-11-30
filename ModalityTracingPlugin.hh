#ifndef MODALITY_TRACING_PLUGIN_HH_
#define MODALITY_TRACING_PLUGIN_HH_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace modality_gz
{
    class TracingPrivate;

    class Tracing:
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPostUpdate
        {
            public: Tracing();

            public: ~Tracing() override;

            public: void Configure(
                            const ignition::gazebo::Entity & entity,
                            const std::shared_ptr < const sdf::Element > & sdf,
                            ignition::gazebo::EntityComponentManager & ecm,
                            ignition::gazebo::EventManager & event_mngr) override;

            public: void PostUpdate(
                            const ignition::gazebo::UpdateInfo &info,
                            const ignition::gazebo::EntityComponentManager &ecm) override;

            private: std::unique_ptr < TracingPrivate > data_ptr;
        };
}

#endif /* MODALITY_TRACING_PLUGIN_HH_ */
