#ifndef MODALITY_MUTATORS_PLUGIN_HH_
#define MODALITY_MUTATORS_PLUGIN_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace modality_gz
{
    class MutatorsPrivate;

    class Mutators:
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemReset
        {
            public: Mutators();

            public: ~Mutators() override;

            public: void Configure(
                            const gz::sim::Entity & entity,
                            const std::shared_ptr < const sdf::Element > & sdf,
                            gz::sim::EntityComponentManager & ecm,
                            gz::sim::EventManager & event_mngr) override;

            public: void PreUpdate(
                            const gz::sim::UpdateInfo &info,
                            gz::sim::EntityComponentManager &ecm) override;

            public: void Reset(
                            const gz::sim::UpdateInfo &info,
                            gz::sim::EntityComponentManager &ecm) override;

            private: std::unique_ptr < MutatorsPrivate > data_ptr;
        };
}

#endif /* MODALITY_MUTATORS_PLUGIN_HH_ */
