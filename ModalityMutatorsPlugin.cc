#include <cstdlib>
#include <chrono>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/math.hh>
#include <gz/common/Uuid.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/ContactSensorData.hh>

#include "ModalityMutatorsPlugin.hh"

#include "modality/error.h"
#include "modality/types.hpp"
#include "modality/runtime.hpp"
#include "modality/tracing_subscriber.hpp"
#include "modality/mutator_interface.hpp"
#include "modality/mutation_client.hpp"

#include "ForceMutator.hh"

GZ_ADD_PLUGIN(
    modality_gz::Mutators,
    gz::sim::System,
    modality_gz::Mutators::ISystemConfigure,
    modality_gz::Mutators::ISystemPreUpdate,
    modality_gz::Mutators::ISystemReset)

using namespace modality_gz;
using namespace modality;

const char ENV_AUTH_TOKEN[] = "MODALITY_AUTH_TOKEN";
const char ENV_MUTATION_URL[] = "MUTATION_PROTOCOL_PARENT_URL";

const char SDF_ENABLED[] = "enabled";
const char SDF_LINK_NAME[] = "link_name";
const char SDF_AUTH_TOKEN[] = "auth_token";
const char SDF_INSECURE_TLS[] = "allow_insecure_tls";
const char SDF_MUTATION_URL[] = "mutation_parent_url";
const char SDF_CONNECT_AUTH_TIMEOUT_MS[] = "connect_auth_timeout_ms";
const char SDF_POLL_TIMEOUT_US[] = "poll_timeout_us";

class modality_gz::MutatorsPrivate
{
    public: void HandleClientError(int err, const char *msg);
    public: void DeInit(void);

    public:
        gz::sim::Entity entity;

        bool enabled{true};
        bool allow_insecure_tls{true};
        uint64_t connect_auth_timeout_ms{5000};
        uint64_t poll_timeout_us{0};

        std::string auth_token;
        std::string mutation_parent_url{"modality-mutation://localhost:14192"};
        std::string model_name;
        std::string link_name;

        struct modality_runtime *rt{NULL};
        struct modality_mutation_client *client{NULL};
        struct modality_mutator force_mutator;
        struct force_mutator_state_s force_mutator_state;
};

void MutatorsPrivate::HandleClientError(int err, const char *msg)
{
    if(err != MODALITY_ERROR_OK)
    {
        gzerr << "A Modality client API call returned a non-zero error code (" << err << ")" << " : " << msg << std::endl;
        this->DeInit();
    }
}

void MutatorsPrivate::DeInit(void)
{
    modality_mutation_client_free(this->client);
    modality_runtime_free(this->rt);
    this->rt = NULL;
    this->client = NULL;
    this->enabled = false;
    this->force_mutator_state.x_pending = false;
    this->force_mutator_state.y_pending = false;
    this->force_mutator_state.z_pending = false;
}

Mutators::Mutators(): data_ptr(std::make_unique < MutatorsPrivate > ())
{
    this->data_ptr->force_mutator_state.x_pending = false;
    this->data_ptr->force_mutator_state.y_pending = false;
    this->data_ptr->force_mutator_state.z_pending = false;
}

Mutators::~Mutators()
{
    this->data_ptr->DeInit();
}

void Mutators::Configure(
        const gz::sim::Entity & entity,
        const std::shared_ptr < const sdf::Element > & sdf,
        gz::sim::EntityComponentManager & ecm,
        gz::sim::EventManager &)
{
    int err;

    this->data_ptr->entity = entity;
    this->data_ptr->model_name = gz::sim::scopedName(entity, ecm, "::", false);

    auto sdf_enabled = sdf->Get<bool>(SDF_ENABLED, true);
    this->data_ptr->enabled = sdf_enabled.first;

    // Load up configs from the SDF
    if(this->data_ptr->enabled)
    {
        if(sdf->HasElement(SDF_LINK_NAME))
        {
            this->data_ptr->link_name = sdf->Get<std::string>(SDF_LINK_NAME);
        }
        else
        {
            gzerr << "Missing key '" << SDF_LINK_NAME << "'" << std::endl;
            this->data_ptr->DeInit();
        }

        if(sdf->HasElement(SDF_AUTH_TOKEN))
        {
            this->data_ptr->auth_token = sdf->Get<std::string>(SDF_AUTH_TOKEN);
        }
        else if(const char *auth_token_env = std::getenv(ENV_AUTH_TOKEN))
        {
            this->data_ptr->auth_token = auth_token_env;
        }
        else
        {
            gzerr << "Missing key '" << SDF_AUTH_TOKEN << "'" << std::endl;
            this->data_ptr->DeInit();
        }

        if(sdf->HasElement(SDF_INSECURE_TLS))
        {
            this->data_ptr->allow_insecure_tls = sdf->Get<bool>(SDF_INSECURE_TLS);
        }

        if(sdf->HasElement(SDF_MUTATION_URL))
        {
            this->data_ptr->mutation_parent_url = sdf->Get<std::string>(SDF_MUTATION_URL);
        }
        else if(const char *mutation_url = std::getenv(ENV_MUTATION_URL))
        {
            this->data_ptr->mutation_parent_url = mutation_url;
        }

        if(sdf->HasElement(SDF_CONNECT_AUTH_TIMEOUT_MS))
        {
            this->data_ptr->connect_auth_timeout_ms = sdf->Get<uint64_t>(SDF_CONNECT_AUTH_TIMEOUT_MS);
        }

        if(sdf->HasElement(SDF_POLL_TIMEOUT_US))
        {
            this->data_ptr->poll_timeout_us = sdf->Get<uint64_t>(SDF_POLL_TIMEOUT_US);
        }
    }

    // Setup the client if config checks out
    if(this->data_ptr->enabled)
    {
        // Enable velocity/acceleration checks for the link, also means it'll have pose too
        gz::sim::Model model{this->data_ptr->entity};
        gz::sim::Link link{model.LinkByName(ecm, this->data_ptr->link_name)};
        link.EnableVelocityChecks(ecm, true);
        link.EnableAccelerationChecks(ecm, true);

        err = modality_tracing_subscriber_init();
        this->data_ptr->HandleClientError(err, "Failed to initialized tracing");

        err = modality_runtime_new(&this->data_ptr->rt);
        this->data_ptr->HandleClientError(err, "Failed to initialized client runtime");

        err = modality_mutation_client_new(this->data_ptr->rt, &this->data_ptr->client);
        this->data_ptr->HandleClientError(err, "Failed to initialized client");
    }

    if(this->data_ptr->enabled)
    {
        err = modality_mutation_client_set_timeout_ms(this->data_ptr->client, this->data_ptr->connect_auth_timeout_ms);
        this->data_ptr->HandleClientError(err, "Failed to set client timeout");

        err = modality_mutation_client_connect(
                this->data_ptr->client,
                this->data_ptr->mutation_parent_url.c_str(),
                this->data_ptr->allow_insecure_tls);
        this->data_ptr->HandleClientError(err, "Failed to connect");

        err = modality_mutation_client_authenticate(this->data_ptr->client, this->data_ptr->auth_token.c_str());
        this->data_ptr->HandleClientError(err, "Failed to authenticate");

        err = modality_mutator_init(&this->data_ptr->force_mutator);
        this->data_ptr->HandleClientError(err, "Failed to init mutator");

        force_mutator_get_iface(&this->data_ptr->force_mutator);
        this->data_ptr->force_mutator.state = (void*) &this->data_ptr->force_mutator_state;
        err = modality_mutation_client_register_mutators(this->data_ptr->client, &this->data_ptr->force_mutator, 1);
        this->data_ptr->HandleClientError(err, "Failed to register mutators");

        err = modality_mutation_client_set_timeout_ms(this->data_ptr->client, this->data_ptr->poll_timeout_us);
        this->data_ptr->HandleClientError(err, "Failed to set client timeout");
    }
}

void Mutators::PreUpdate(
        const gz::sim::UpdateInfo &info,
        gz::sim::EntityComponentManager &ecm)
{
    int err;
    bool apply_force = false;
    gz::math::Vector3d force{0, 0, 0};

    if(!this->data_ptr->enabled)
    {
        return;
    }

    err = modality_mutation_client_poll(this->data_ptr->client);
    this->data_ptr->HandleClientError(err, "Failed to poll client");

    if(this->data_ptr->force_mutator_state.x_pending)
    {
        apply_force = true;
        force.X() = this->data_ptr->force_mutator_state.x;
    }
    if(this->data_ptr->force_mutator_state.y_pending)
    {
        apply_force = true;
        force.Y() = this->data_ptr->force_mutator_state.y;
    }
    if(this->data_ptr->force_mutator_state.z_pending)
    {
        apply_force = true;
        force.Z() = this->data_ptr->force_mutator_state.z;
    }

    if(apply_force)
    {
        gz::sim::Model model{this->data_ptr->entity};
        gz::sim::Link link{model.LinkByName(ecm, this->data_ptr->link_name)};

        gzdbg << "Apply world force mutation: " << force << std::endl;
        link.AddWorldForce(ecm, force);

        this->data_ptr->force_mutator_state.x_pending = false;
        this->data_ptr->force_mutator_state.y_pending = false;
        this->data_ptr->force_mutator_state.z_pending = false;
    }
}

void Mutators::Reset(
        const gz::sim::UpdateInfo &info,
        gz::sim::EntityComponentManager &ecm)
{
    this->data_ptr->force_mutator_state.x_pending = false;
    this->data_ptr->force_mutator_state.y_pending = false;
    this->data_ptr->force_mutator_state.z_pending = false;
}
