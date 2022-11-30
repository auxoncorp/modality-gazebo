#include <cstdlib>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/math.hh>
#include <ignition/common/Uuid.hh>

#include "ModalityTracingPlugin.hh"

#include "modality/error.h"
#include "modality/types.hpp"
#include "modality/runtime.hpp"
#include "modality/tracing_subscriber.hpp"
#include "modality/ingest_client.hpp"

IGNITION_ADD_PLUGIN(
    modality_gz::Tracing,
    ignition::gazebo::System,
    modality_gz::Tracing::ISystemConfigure,
    modality_gz::Tracing::ISystemPostUpdate)

using namespace modality_gz;
using namespace modality;

#define NS_PER_SEC (1000000000ULL)

const char EVENT_NAME_POSE[] = "pose";
const char EVENT_NAME_LINEAR_VEL[] = "linear_velocity";
const char EVENT_NAME_LINEAR_ACCEL[] = "linear_acceleration";

#define TID_IDX_RUN_ID (0)
#define TID_IDX_NAME (1)
#define NUM_TIMELINE_ATTRS (2)
static const char *TIMELINE_ATTR_KEYS[] =
{
    "timeline.run_id",
    "timeline.name",
    // TODO
    // time domain
    // internal.gazebo.model_name
    // internal.gazebo.link_name
    // ...
};

#define EID_IDX_NAME (0)
#define EID_IDX_TIMESTAMP (1)
#define EID_IDX_X (2)
#define EID_IDX_Y (3)
#define EID_IDX_Z (4)
#define EID_IDX_ROLL (5)
#define EID_IDX_PITCH (6)
#define EID_IDX_YAW (7)

#define NUM_EVENT_ATTRS (8)
#define NUM_POSE_EVENT_ATTRS_POSE (2 + 3 + 3)
#define NUM_EVENT_ATTRS_LINEAR_VEL (2 + 3)
#define NUM_EVENT_ATTRS_LINEAR_ACCEL (2 + 3)
static const char *EVENT_ATTR_KEYS[] =
{
    "event.name",
    "event.timestamp",
    "event.x",
    "event.y",
    "event.z",
    "event.roll",
    "event.pitch",
    "event.yaw",
};

class modality_gz::TracingPrivate
{
    public: void HandleClientError(int err, const char *msg);
    public: void DeInit(void);

    public:
        ignition::gazebo::Entity entity;
        std::chrono::steady_clock::duration current_time;
        bool tracing_enabled;
        bool trace_pose;
        bool trace_linear_accel;
        bool trace_linear_vel;
        bool allow_insecure_tls;
        std::string auth_token;
        std::string timeline_name;
        std::string modalityd_url;
        std::string link_name;
        std::string run_id;
        uint64_t ordering;
        struct modality_runtime *rt;
        struct modality_ingest_client *client;
        modality_timeline_id tid;
        modality_attr timeline_attrs[NUM_TIMELINE_ATTRS];
        modality_attr event_attrs[NUM_EVENT_ATTRS];
};

void TracingPrivate::HandleClientError(int err, const char *msg)
{
    if(err != MODALITY_ERROR_OK)
    {
        ignerr << "A Modality client API call returned a non-zero error code (" << err << ")" << " : " << msg << std::endl;
        this->DeInit();
    }
}

void TracingPrivate::DeInit(void)
{
    if(this->client)
    {
        (void) modality_ingest_client_close_timeline(this->client);
    }
    modality_ingest_client_free(this->client);
    modality_runtime_free(this->rt);
    this->rt = NULL;
    this->client = NULL;
    this->tracing_enabled = false;
}

Tracing::Tracing(): data_ptr(std::make_unique < TracingPrivate > ())
{
    this->data_ptr->tracing_enabled = true;
    this->data_ptr->trace_pose = true;
    this->data_ptr->trace_linear_accel = true;
    this->data_ptr->trace_linear_vel = true;
    this->data_ptr->rt = NULL;
    this->data_ptr->client = NULL;
    this->data_ptr->ordering = 0;
}

Tracing::~Tracing()
{
    this->data_ptr->DeInit();
}

void Tracing::Configure(
  const ignition::gazebo::Entity & entity,
  const std::shared_ptr < const sdf::Element > & sdf,
  ignition::gazebo::EntityComponentManager & ecm,
  ignition::gazebo::EventManager &)
{
    int err;
    int i;

    this->data_ptr->entity = entity;

    auto sdf_tracing_enabled = sdf->Get<bool>("enabled", true);
    this->data_ptr->tracing_enabled = sdf_tracing_enabled.first;

    // Load up configs from the SDF
    if(this->data_ptr->tracing_enabled)
    {
        auto sdf_link_name = sdf->Get<std::string>("link_name", "");
        if(!sdf_link_name.second)
        {
            ignerr << "Missing key 'link_name'" << std::endl;
            this->data_ptr->DeInit();
        }
        else
        {
            this->data_ptr->link_name = sdf_link_name.first;
        }
        
        if(const char *auth_token_env = std::getenv("MODALITY_AUTH_TOKEN"))
        {
            this->data_ptr->auth_token = auth_token_env;
        }
        else
        {
            auto sdf_auth_token = sdf->Get<std::string>("auth_token", "");
            if(!sdf_auth_token.second)
            {
                ignerr << "Missing key 'auth_token'" << std::endl;
                this->data_ptr->DeInit();
            }
            else
            {
                this->data_ptr->auth_token = sdf_auth_token.first;
            }
        }

        auto sdf_timeline_name = sdf->Get<std::string>("timeline_name", "");
        if(sdf_timeline_name.second)
        {
            this->data_ptr->timeline_name = sdf_timeline_name.first;
        }
        else
        {
            // Use scoped entity name for the timeline name if not explicitly provided
            this->data_ptr->timeline_name = ignition::gazebo::scopedName(entity, ecm, "::", false);
        }

        auto sdf_allow_insecure_tls = sdf->Get<bool>("allow_insecure_tls", true);
        this->data_ptr->allow_insecure_tls = sdf_allow_insecure_tls.first;

        auto sdf_modalityd_url = sdf->Get<std::string>("modalityd_url", "modality-ingest://localhost:14182");
        this->data_ptr->modalityd_url = sdf_modalityd_url.first;

        auto sdf_trace_pose = sdf->Get<bool>("pose", true);
        this->data_ptr->trace_pose = sdf_trace_pose.first;

        auto sdf_trace_linear_accel = sdf->Get<bool>("linear_acceleration", true);
        this->data_ptr->trace_linear_accel = sdf_trace_linear_accel.first;

        auto sdf_trace_linear_vel = sdf->Get<bool>("linear_velocity", true);
        this->data_ptr->trace_linear_vel = sdf_trace_linear_vel.first;
    }

    // Setup the client if config checks out
    if(this->data_ptr->tracing_enabled)
    {
        // Enable velocity/acceleration checks for the link, also means it'll have pose too
        ignition::gazebo::Model model{this->data_ptr->entity};
        ignition::gazebo::Link link{model.LinkByName(ecm, this->data_ptr->link_name)};
        link.EnableVelocityChecks(ecm, true);
        link.EnableAccelerationChecks(ecm, true);

        err = modality_runtime_new(&this->data_ptr->rt);
        this->data_ptr->HandleClientError(err, "Failed to initialized client runtime");

        err = modality_ingest_client_new(this->data_ptr->rt, &this->data_ptr->client);
        this->data_ptr->HandleClientError(err, "Failed to initialized client");
    }

    if(this->data_ptr->tracing_enabled)
    {
        err = modality_ingest_client_connect(
                this->data_ptr->client,
                this->data_ptr->modalityd_url.c_str(),
                this->data_ptr->allow_insecure_tls);
        this->data_ptr->HandleClientError(err, "Failed to connect");

        err = modality_ingest_client_authenticate(this->data_ptr->client, this->data_ptr->auth_token.c_str());
        this->data_ptr->HandleClientError(err, "Failed to authenticate");

        err = modality_timeline_id_init(&this->data_ptr->tid);
        this->data_ptr->HandleClientError(err, "Failed to initialize timeline ID");

        err = modality_ingest_client_open_timeline(this->data_ptr->client, &this->data_ptr->tid);
        this->data_ptr->HandleClientError(err, "Failed to open timeline");

        for(i = 0; i < NUM_TIMELINE_ATTRS; i += 1)
        {
            err = modality_ingest_client_declare_attr_key(
                    this->data_ptr->client,
                    TIMELINE_ATTR_KEYS[i],
                    &this->data_ptr->timeline_attrs[i].key);
            this->data_ptr->HandleClientError(err, "Failed to declare timeline attribute key");
        }

        for(i = 0; i < NUM_EVENT_ATTRS; i += 1)
        {
            err = modality_ingest_client_declare_attr_key(
                    this->data_ptr->client,
                    EVENT_ATTR_KEYS[i],
                    &this->data_ptr->event_attrs[i].key);
            this->data_ptr->HandleClientError(err, "Failed to declare event attribute key");
        }

        err = modality_attr_val_set_string(&this->data_ptr->timeline_attrs[TID_IDX_NAME].val, this->data_ptr->timeline_name.c_str());
        this->data_ptr->HandleClientError(err, "Failed to set timeline attribute value");

        if(const char *run_id_env = std::getenv("MODALITY_RUN_ID"))
        {
            this->data_ptr->run_id = run_id_env;
        }
        else
        {
            auto uuid_run_id = ignition::common::Uuid();
            this->data_ptr->run_id = uuid_run_id.String();
        }
        err = modality_attr_val_set_string(&this->data_ptr->timeline_attrs[TID_IDX_RUN_ID].val, this->data_ptr->run_id.c_str());
        this->data_ptr->HandleClientError(err, "Failed to set timeline attribute value");

        err = modality_ingest_client_timeline_metadata(this->data_ptr->client, this->data_ptr->timeline_attrs, NUM_TIMELINE_ATTRS);
        this->data_ptr->HandleClientError(err, "Failed to send timeline metadata");
    }
}

void Tracing::PostUpdate(
        const ignition::gazebo::UpdateInfo &info,
        const ignition::gazebo::EntityComponentManager &ecm)
{
    int err;

    bool not_tracing = !this->data_ptr->tracing_enabled || info.paused;
    bool no_data_selected = !(this->data_ptr->trace_pose || this->data_ptr->trace_linear_vel || this->data_ptr->trace_linear_accel);

    if(not_tracing || no_data_selected || (this->data_ptr->current_time == info.simTime))
    {
        // Not tracing or paused
        return;
    }

    ignition::gazebo::Model model{this->data_ptr->entity};
    ignition::gazebo::Link link{model.LinkByName(ecm, this->data_ptr->link_name)};

    auto sec_nsec = ignition::math::durationToSecNsec(info.simTime);
    uint64_t ts_ns = ((uint64_t) sec_nsec.first) * NS_PER_SEC;
    ts_ns += (uint64_t) sec_nsec.second;
    this->data_ptr->current_time = info.simTime;

    err = modality_attr_val_set_timestamp(&this->data_ptr->event_attrs[EID_IDX_TIMESTAMP].val, ts_ns);
    this->data_ptr->HandleClientError(err, "Failed to set event timestamp attribute value");

    if(this->data_ptr->trace_pose)
    {
        if(auto maybe_pose = link.WorldPose(ecm))
        {
            auto pose = maybe_pose.value();
            err = modality_attr_val_set_string(&this->data_ptr->event_attrs[EID_IDX_NAME].val, EVENT_NAME_POSE);
            this->data_ptr->HandleClientError(err, "Failed to set timeline attribute value");

            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_X].val, pose.X());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Y].val, pose.Y());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Z].val, pose.Z());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");

            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_ROLL].val, pose.Roll());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_PITCH].val, pose.Pitch());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_YAW].val, pose.Yaw());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");

            err = modality_ingest_client_event(
                    this->data_ptr->client,
                    this->data_ptr->ordering,
                    0,
                    this->data_ptr->event_attrs,
                    NUM_POSE_EVENT_ATTRS_POSE);
            this->data_ptr->HandleClientError(err, "Failed to send event");

            this->data_ptr->ordering += 1;
        }
        else
        {
            ignwarn << "Link entity [" << this->data_ptr->entity << " : " << this->data_ptr->link_name << "] doesn't have a pose component" << std::endl;
        }
    }

    if(this->data_ptr->trace_linear_vel)
    {
        if(auto maybe_lin_vel = link.WorldLinearVelocity(ecm))
        {
            auto vel = maybe_lin_vel.value();
            err = modality_attr_val_set_string(&this->data_ptr->event_attrs[EID_IDX_NAME].val, EVENT_NAME_LINEAR_VEL);
            this->data_ptr->HandleClientError(err, "Failed to set timeline attribute value");

            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_X].val, vel.X());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Y].val, vel.Y());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Z].val, vel.Z());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");

            err = modality_ingest_client_event(
                    this->data_ptr->client,
                    this->data_ptr->ordering,
                    0,
                    this->data_ptr->event_attrs,
                    NUM_EVENT_ATTRS_LINEAR_VEL);
            this->data_ptr->HandleClientError(err, "Failed to send event");

            this->data_ptr->ordering += 1;
        }
        else
        {
            ignwarn << "Link entity [" << this->data_ptr->entity << " : " << this->data_ptr->link_name << "] doesn't have a linear velocity component" << std::endl;
        }
    }

    if(this->data_ptr->trace_linear_accel)
    {
        if(auto maybe_lin_accel = link.WorldLinearAcceleration(ecm))
        {
            auto accel = maybe_lin_accel.value();
            err = modality_attr_val_set_string(&this->data_ptr->event_attrs[EID_IDX_NAME].val, EVENT_NAME_LINEAR_ACCEL);
            this->data_ptr->HandleClientError(err, "Failed to set timeline attribute value");

            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_X].val, accel.X());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Y].val, accel.Y());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Z].val, accel.Z());
            this->data_ptr->HandleClientError(err, "Failed to set event attribute value");

            err = modality_ingest_client_event(
                    this->data_ptr->client,
                    this->data_ptr->ordering,
                    0,
                    this->data_ptr->event_attrs,
                    NUM_EVENT_ATTRS_LINEAR_ACCEL);
            this->data_ptr->HandleClientError(err, "Failed to send event");

            this->data_ptr->ordering += 1;
        }
        else
        {
            ignwarn << "Link entity [" << this->data_ptr->entity << " : " << this->data_ptr->link_name << "] doesn't have a linear acceleration component" << std::endl;
        }
    }
}