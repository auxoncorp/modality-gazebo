#ifndef MODALITY_TRACING_PLUGIN_HH_
#define MODALITY_TRACING_PLUGIN_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace modality_gz
{
    class TracingPrivate;

    class Tracing:
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPostUpdate
        {
            public: Tracing();

            public: ~Tracing() override;

            public: void Configure(
                            const gz::sim::Entity & entity,
                            const std::shared_ptr < const sdf::Element > & sdf,
                            gz::sim::EntityComponentManager & ecm,
                            gz::sim::EventManager & event_mngr) override;

            public: void PostUpdate(
                            const gz::sim::UpdateInfo &info,
                            const gz::sim::EntityComponentManager &ecm) override;

            private: std::unique_ptr < TracingPrivate > data_ptr;
        };
}

#endif /* MODALITY_TRACING_PLUGIN_HH_ */
