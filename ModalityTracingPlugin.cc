#include <cstdlib>
#include <vector>
#include <mutex>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/math.hh>
#include <gz/common/Uuid.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/ContactSensorData.hh>

// TODO - switch over to transport::ProtoMsg after prototyping
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/twist.pb.h>

#include "ModalityTracingPlugin.hh"

#include "modality/error.h"
#include "modality/types.hpp"
#include "modality/runtime.hpp"
#include "modality/tracing_subscriber.hpp"
#include "modality/ingest_client.hpp"

GZ_ADD_PLUGIN(
    modality_gz::Tracing,
    gz::sim::System,
    modality_gz::Tracing::ISystemConfigure,
    modality_gz::Tracing::ISystemPostUpdate)

using namespace modality_gz;
using namespace modality;

#define NS_PER_SEC (1000000000ULL)

const char TIME_DOMAIN[] = "gazebo-simulator-clock";
const char CLOCK_STYLE[] = "relative";

const char ENV_AUTH_TOKEN[] = "MODALITY_AUTH_TOKEN";
const char ENV_RUN_ID[] = "MODALITY_RUN_ID";

const char SDF_LINK_NAME[] = "link_name";
const char SDF_AUTH_TOKEN[] = "auth_token";
const char SDF_TIMELINE_NAME[] = "timeline_name";
const char SDF_INSECURE_TLS[] = "allow_insecure_tls";
const char SDF_MODALITYD_URL[] = "modalityd_url";
const char SDF_STEP_SIZE[] = "step_size";
const char SDF_SUBSCRIBED_INTERACTION_TOPICS[] = "subscribed_interaction_topics";
const char SDF_PUBLISHED_INTERACTION_TOPICS[] = "published_interaction_topics";
const char SDF_TRACE_POSE[] = "pose";
const char SDF_TRACE_LIN_ACCEL[] = "linear_acceleration";
const char SDF_TRACE_LIN_VEL[] = "linear_velocity";
const char SDF_TRACE_CONTACT_COLLISION[] = "contact_collision";

const char EVENT_NAME_POSE[] = "pose";
const char EVENT_NAME_LINEAR_VEL[] = "linear_velocity";
const char EVENT_NAME_LINEAR_ACCEL[] = "linear_acceleration";
const char EVENT_NAME_CONTACT[] = "contact";
const char EVENT_NAME_RECV_MSG[] = "receive_message";
const char EVENT_NAME_PUB_MSG[] = "publish_message";

const char ERR_TIMELINE_ATTR_VAL[] = "Failed to set timeline attribute value";
const char ERR_EVENT_ATTR_VAL[] = "Failed to set event attribute value";
const char ERR_EVENT_SEND[] = "Failed to send event";

#define TID_IDX_RUN_ID (0)
#define TID_IDX_NAME (1)
#define TID_IDX_TIME_DOMAIN (2)
#define TID_IDX_CLOCK_STYLE (3)
#define TID_IDX_MODEL_NAME (4)
#define TID_IDX_MODEL_ENTITY (5)
#define TID_IDX_LINK_NAME (6)
#define TID_IDX_LINK_ENTITY (7)
#define TID_IDX_STEP_SIZE (8)
#define NUM_TIMELINE_ATTRS (9)
static const char *TIMELINE_ATTR_KEYS[] =
{
    "timeline.run_id",
    "timeline.name",
    "timeline.time_domain",
    "timeline.clock_style",
    "timeline.internal.gazebo.model.name",
    "timeline.internal.gazebo.model.entity",
    "timeline.internal.gazebo.link.name",
    "timeline.internal.gazebo.link.entity",
    "timeline.internal.gazebo.step_size",
};

#define EID_IDX_COLLISION_NAME (0)
#define EID_IDX_COLLISION_ENTITY (1)
#define EID_IDX_NAME (2)
#define EID_IDX_TIMESTAMP (3)
#define EID_IDX_SIM_TIME (4)
#define EID_IDX_WALL_CLOCK_TIME (5)
#define EID_IDX_X (6)
#define EID_IDX_Y (7)
#define EID_IDX_Z (8)
#define EID_IDX_ROLL (9)
#define EID_IDX_PITCH (10)
#define EID_IDX_YAW (11)

#define NUM_EVENT_ATTRS (12)
// First 4 event attrs always name, timestmap, gz_wct, gz_st
#define NUM_EVENT_ATTRS_POSE (4 + 3 + 3)
#define NUM_EVENT_ATTRS_LINEAR_VEL (4 + 3)
#define NUM_EVENT_ATTRS_LINEAR_ACCEL (4 + 3)
#define NUM_EVENT_ATTRS_CONTACT (4 + 2)

// Ordered such that 0..=5, 2..=8, and 2..=11 can be contiguous arrays
static const char *EVENT_ATTR_KEYS[] =
{
    "event.collision.name",
    "event.collision.entity",
    "event.name",
    "event.timestamp",
    "event.internal.gazebo.simulation_time",
    "event.internal.gazebo.wall_clock_time",
    "event.x",
    "event.y",
    "event.z",
    "event.roll",
    "event.pitch",
    "event.yaw",
};

#define EID_ALT_IDX_SEQNUM (0)
#define EID_ALT_IDX_TOPIC_NAME (1)
#define NUM_EVENT_ATTRS_ALT (6)
static const char *EVENT_ATTR_KEYS_ALT[] =
{
    "event.seqnum",
    "event.topic",
    "event.name",
    "event.timestamp",
    "event.internal.gazebo.simulation_time",
    "event.internal.gazebo.wall_clock_time",
};

static inline uint64_t dur_to_ns(std::chrono::steady_clock::duration dur)
{
    auto sec_nsec = gz::math::durationToSecNsec(dur);
    uint64_t ts_ns = ((uint64_t) sec_nsec.first) * NS_PER_SEC;
    ts_ns += (uint64_t) sec_nsec.second;
    return ts_ns;
}

class modality_gz::TracingPrivate
{
    public: void HandleClientError(int err, const char *msg);
    public: void DeInit(void);

    public:
        gz::transport::Node node;
        gz::sim::Entity entity;
        gz::sim::Entity collision_entity{gz::sim::kNullEntity};
        std::chrono::steady_clock::duration current_time;

        std::string pub_interactions_topic;
        std::vector<gz::msgs::IMU> pub_interactions_queue;
        std::mutex pub_interactions_queue_mutex;

        std::string sub_interactions_topic;
        std::vector<gz::msgs::Twist> sub_interactions_queue;
        std::mutex sub_interactions_queue_mutex;

        bool tracing_enabled{true};
        bool trace_pose{true};
        bool trace_linear_accel{true};
        bool trace_linear_vel{true};
        bool trace_contact_collision{false};
        bool allow_insecure_tls{true};

        std::string auth_token;
        std::string timeline_name;
        std::string modalityd_url{"modality-ingest://localhost:14182"};
        std::string model_name;
        std::string link_name;
        std::string run_id;

        struct modality_big_int link_entity;
        struct modality_big_int model_entity;
        uint64_t ordering{0};
        struct modality_runtime *rt{NULL};
        struct modality_ingest_client *client{NULL};

        modality_timeline_id tid;
        modality_attr timeline_attrs[NUM_TIMELINE_ATTRS];
        modality_attr event_attrs[NUM_EVENT_ATTRS];
        modality_attr event_attrs_alt[NUM_EVENT_ATTRS_ALT];
};

void TracingPrivate::HandleClientError(int err, const char *msg)
{
    if(err != MODALITY_ERROR_OK)
    {
        gzerr << "A Modality client API call returned a non-zero error code (" << err << ")" << " : " << msg << std::endl;
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
}

Tracing::~Tracing()
{
    this->data_ptr->DeInit();
}

void Tracing::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr < const sdf::Element > & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager &)
{
    int err;
    int i;
    double step_size;

    auto pub_msg_cb = std::function<void(const gz::msgs::IMU &)>(
        [this](const gz::msgs::IMU &msg)
        {
            std::lock_guard<std::mutex> lock(this->data_ptr->pub_interactions_queue_mutex);
            this->data_ptr->pub_interactions_queue.push_back(msg);
        });

    auto sub_msg_cb = std::function<void(const gz::msgs::Twist &)>(
        [this](const gz::msgs::Twist &msg)
        {
            std::lock_guard<std::mutex> lock(this->data_ptr->sub_interactions_queue_mutex);
            this->data_ptr->sub_interactions_queue.push_back(msg);
        });

    this->data_ptr->entity = entity;
    this->data_ptr->model_name = gz::sim::scopedName(entity, ecm, "::", false);

    auto sdf_tracing_enabled = sdf->Get<bool>("enabled", true);
    this->data_ptr->tracing_enabled = sdf_tracing_enabled.first;

    // Load up configs from the SDF
    if(this->data_ptr->tracing_enabled)
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

        if(const char *auth_token_env = std::getenv(ENV_AUTH_TOKEN))
        {
            this->data_ptr->auth_token = auth_token_env;
        }
        else if(sdf->HasElement(SDF_AUTH_TOKEN))
        {
            this->data_ptr->auth_token = sdf->Get<std::string>(SDF_AUTH_TOKEN);
        }
        else
        {
            gzerr << "Missing key '" << SDF_AUTH_TOKEN << "'" << std::endl;
            this->data_ptr->DeInit();
        }

        if(sdf->HasElement(SDF_TIMELINE_NAME))
        {
            this->data_ptr->timeline_name = sdf->Get<std::string>(SDF_TIMELINE_NAME);
        }
        else
        {
            // Use scoped entity name for the timeline name if not explicitly provided
            this->data_ptr->timeline_name = gz::sim::scopedName(entity, ecm, "::", false);
        }

        if(sdf->HasElement(SDF_INSECURE_TLS))
        {
            this->data_ptr->allow_insecure_tls = sdf->Get<bool>(SDF_INSECURE_TLS);
        }

        if(sdf->HasElement(SDF_MODALITYD_URL))
        {
            this->data_ptr->modalityd_url = sdf->Get<std::string>(SDF_MODALITYD_URL);
        }

        if(sdf->HasElement(SDF_TRACE_POSE))
        {
            this->data_ptr->trace_pose = sdf->Get<bool>(SDF_TRACE_POSE);
        }

        if(sdf->HasElement(SDF_TRACE_LIN_ACCEL))
        {
            this->data_ptr->trace_linear_accel = sdf->Get<bool>(SDF_TRACE_LIN_ACCEL);
        }

        if(sdf->HasElement(SDF_TRACE_LIN_VEL))
        {
            this->data_ptr->trace_linear_vel = sdf->Get<bool>(SDF_TRACE_LIN_VEL);
        }

        if(sdf->HasElement(SDF_TRACE_CONTACT_COLLISION))
        {
            this->data_ptr->trace_contact_collision = sdf->Get<bool>(SDF_TRACE_CONTACT_COLLISION);
        }

        auto default_step_size = sdf->Get<double>(SDF_STEP_SIZE, 0.001);
        step_size = default_step_size.first;

        auto pub_topic_name = gz::transport::TopicUtils::AsValidTopic(sdf->Get<std::string>(SDF_PUBLISHED_INTERACTION_TOPICS));
        if(!pub_topic_name.empty())
        {
            this->data_ptr->pub_interactions_topic = pub_topic_name;
            if(!this->data_ptr->node.Subscribe(pub_topic_name, pub_msg_cb))
            {
                gzerr << "Subscriber could not be created for topic ["
                    << pub_topic_name << "] with message type [IMU]" << std::endl;
                return;
            }
        }

        auto sub_topic_name = gz::transport::TopicUtils::AsValidTopic(sdf->Get<std::string>(SDF_SUBSCRIBED_INTERACTION_TOPICS));
        if(!sub_topic_name.empty())
        {
            this->data_ptr->sub_interactions_topic = sub_topic_name;
            if(!this->data_ptr->node.Subscribe(sub_topic_name, sub_msg_cb))
            {
                gzerr << "Subscriber could not be created for topic ["
                    << pub_topic_name << "] with message type [Twist]" << std::endl;
                return;
            }
        }
    }

    // Setup the client if config checks out
    if(this->data_ptr->tracing_enabled)
    {
        // Enable velocity/acceleration checks for the link, also means it'll have pose too
        gz::sim::Model model{this->data_ptr->entity};
        gz::sim::Link link{model.LinkByName(ecm, this->data_ptr->link_name)};
        link.EnableVelocityChecks(ecm, true);
        link.EnableAccelerationChecks(ecm, true);

        // Get entity handle to collision, if present
        if(this->data_ptr->trace_contact_collision)
        {
            this->data_ptr->collision_entity = link.CollisionByName(ecm, "collision");
            if(this->data_ptr->collision_entity == gz::sim::kNullEntity)
            {
                this->data_ptr->trace_contact_collision = false;
                gzerr << "Could not find contact sensor with collision on link '" << this->data_ptr->link_name << "'" << std::endl;
            }
        }

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

        for(i = 0; i < NUM_EVENT_ATTRS_ALT; i += 1)
        {
            err = modality_ingest_client_declare_attr_key(
                    this->data_ptr->client,
                    EVENT_ATTR_KEYS_ALT[i],
                    &this->data_ptr->event_attrs_alt[i].key);
            this->data_ptr->HandleClientError(err, "Failed to declare alt event attribute key");
        }

        err = modality_attr_val_set_string(&this->data_ptr->timeline_attrs[TID_IDX_NAME].val, this->data_ptr->timeline_name.c_str());
        this->data_ptr->HandleClientError(err, ERR_TIMELINE_ATTR_VAL);

        if(const char *run_id_env = std::getenv(ENV_RUN_ID))
        {
            this->data_ptr->run_id = run_id_env;
        }
        else
        {
            auto uuid_run_id = gz::common::Uuid();
            this->data_ptr->run_id = uuid_run_id.String();
        }
        err = modality_attr_val_set_string(&this->data_ptr->timeline_attrs[TID_IDX_RUN_ID].val, this->data_ptr->run_id.c_str());
        this->data_ptr->HandleClientError(err, ERR_TIMELINE_ATTR_VAL);

        err = modality_attr_val_set_string(&this->data_ptr->timeline_attrs[TID_IDX_TIME_DOMAIN].val, TIME_DOMAIN);
        this->data_ptr->HandleClientError(err, ERR_TIMELINE_ATTR_VAL);

        err = modality_attr_val_set_string(&this->data_ptr->timeline_attrs[TID_IDX_CLOCK_STYLE].val, CLOCK_STYLE);
        this->data_ptr->HandleClientError(err, ERR_TIMELINE_ATTR_VAL);

        err = modality_attr_val_set_string(&this->data_ptr->timeline_attrs[TID_IDX_MODEL_NAME].val, this->data_ptr->model_name.c_str());
        this->data_ptr->HandleClientError(err, ERR_TIMELINE_ATTR_VAL);

        err = modality_big_int_set(&this->data_ptr->model_entity, this->data_ptr->entity, 0);
        this->data_ptr->HandleClientError(err, "Failed to set model entity big int value");
        err = modality_attr_val_set_big_int(&this->data_ptr->timeline_attrs[TID_IDX_MODEL_ENTITY].val, &this->data_ptr->model_entity);
        this->data_ptr->HandleClientError(err, ERR_TIMELINE_ATTR_VAL);

        err = modality_attr_val_set_string(&this->data_ptr->timeline_attrs[TID_IDX_LINK_NAME].val, this->data_ptr->link_name.c_str());
        this->data_ptr->HandleClientError(err, ERR_TIMELINE_ATTR_VAL);

        gz::sim::Model model{this->data_ptr->entity};
        auto link_entity = model.LinkByName(ecm, this->data_ptr->link_name);
        err = modality_big_int_set(&this->data_ptr->link_entity, link_entity, 0);
        this->data_ptr->HandleClientError(err, "Failed to set link entity big int value");
        err = modality_attr_val_set_big_int(&this->data_ptr->timeline_attrs[TID_IDX_LINK_ENTITY].val, &this->data_ptr->link_entity);
        this->data_ptr->HandleClientError(err, ERR_TIMELINE_ATTR_VAL);

        err = modality_attr_val_set_float(&this->data_ptr->timeline_attrs[TID_IDX_STEP_SIZE].val, step_size);
        this->data_ptr->HandleClientError(err, ERR_TIMELINE_ATTR_VAL);

        err = modality_ingest_client_timeline_metadata(this->data_ptr->client, this->data_ptr->timeline_attrs, NUM_TIMELINE_ATTRS);
        this->data_ptr->HandleClientError(err, "Failed to send timeline metadata");
    }
}

void Tracing::PostUpdate(
        const gz::sim::UpdateInfo &info,
        const gz::sim::EntityComponentManager &ecm)
{
    int err;

    bool not_tracing = !this->data_ptr->tracing_enabled || info.paused;
    bool no_data_selected = !(this->data_ptr->trace_pose || this->data_ptr->trace_linear_vel || this->data_ptr->trace_linear_accel);

    if(not_tracing || no_data_selected || (this->data_ptr->current_time == info.simTime))
    {
        // Not tracing or paused
        return;
    }

    gz::sim::Model model{this->data_ptr->entity};
    gz::sim::Link link{model.LinkByName(ecm, this->data_ptr->link_name)};

    bool model_is_static = model.Static(ecm);

    this->data_ptr->current_time = info.simTime;
    const uint64_t sim_time_ns = dur_to_ns(info.simTime);
    const uint64_t wall_clock_time_ns = dur_to_ns(info.realTime);

    {
        std::lock_guard<std::mutex> lock1(this->data_ptr->sub_interactions_queue_mutex);
        for(auto msg = std::begin(this->data_ptr->sub_interactions_queue); msg != std::end(this->data_ptr->sub_interactions_queue);)
        {
            std::string native_seqnum = msg->header().data(0).value(0);
            int64_t seqnum = stoi(native_seqnum);
            uint64_t ts_ns = sim_time_ns; // TODO

            err = modality_attr_val_set_timestamp(&this->data_ptr->event_attrs_alt[EID_IDX_TIMESTAMP].val, ts_ns);
            this->data_ptr->HandleClientError(err, "Failed to set event timestamp attribute value");
            err = modality_attr_val_set_timestamp(&this->data_ptr->event_attrs_alt[EID_IDX_SIM_TIME].val, ts_ns);
            this->data_ptr->HandleClientError(err, "Failed to set event sim time attribute value");
            err = modality_attr_val_set_timestamp(&this->data_ptr->event_attrs_alt[EID_IDX_WALL_CLOCK_TIME].val, wall_clock_time_ns);
            this->data_ptr->HandleClientError(err, "Failed to set event wall clock time attribute value");

            err = modality_attr_val_set_string(&this->data_ptr->event_attrs_alt[EID_IDX_NAME].val, EVENT_NAME_RECV_MSG);
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_attr_val_set_string(
                    &this->data_ptr->event_attrs_alt[EID_ALT_IDX_TOPIC_NAME].val,
                    this->data_ptr->sub_interactions_topic.c_str());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_attr_val_set_integer(&this->data_ptr->event_attrs_alt[EID_ALT_IDX_SEQNUM].val, seqnum);
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_ingest_client_event(
                    this->data_ptr->client,
                    this->data_ptr->ordering,
                    0,
                    &this->data_ptr->event_attrs_alt[0],
                    NUM_EVENT_ATTRS_ALT);
            this->data_ptr->HandleClientError(err, ERR_EVENT_SEND);

            this->data_ptr->ordering += 1;
            msg = this->data_ptr->sub_interactions_queue.erase(msg);
        }
    }

    {
        std::lock_guard<std::mutex> lock1(this->data_ptr->pub_interactions_queue_mutex);
        for(auto msg = std::begin(this->data_ptr->pub_interactions_queue); msg != std::end(this->data_ptr->pub_interactions_queue);)
        {
            std::string native_seqnum = msg->header().data(1).value(0);
            int64_t seqnum = stoi(native_seqnum);
            auto timestamp = msg->header().stamp();
            uint64_t ts_ns = ((uint64_t) timestamp.sec()) * NS_PER_SEC;
            ts_ns += (uint64_t) timestamp.nsec();

            err = modality_attr_val_set_timestamp(&this->data_ptr->event_attrs_alt[EID_IDX_TIMESTAMP].val, ts_ns);
            this->data_ptr->HandleClientError(err, "Failed to set event timestamp attribute value");
            err = modality_attr_val_set_timestamp(&this->data_ptr->event_attrs_alt[EID_IDX_SIM_TIME].val, ts_ns);
            this->data_ptr->HandleClientError(err, "Failed to set event sim time attribute value");
            err = modality_attr_val_set_timestamp(&this->data_ptr->event_attrs_alt[EID_IDX_WALL_CLOCK_TIME].val, wall_clock_time_ns);
            this->data_ptr->HandleClientError(err, "Failed to set event wall clock time attribute value");

            err = modality_attr_val_set_string(&this->data_ptr->event_attrs_alt[EID_IDX_NAME].val, EVENT_NAME_PUB_MSG);
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_attr_val_set_string(
                    &this->data_ptr->event_attrs_alt[EID_ALT_IDX_TOPIC_NAME].val,
                    this->data_ptr->pub_interactions_topic.c_str());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_attr_val_set_integer(&this->data_ptr->event_attrs_alt[EID_ALT_IDX_SEQNUM].val, seqnum);
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_ingest_client_event(
                    this->data_ptr->client,
                    this->data_ptr->ordering,
                    0,
                    &this->data_ptr->event_attrs_alt[0],
                    NUM_EVENT_ATTRS_ALT);
            this->data_ptr->HandleClientError(err, ERR_EVENT_SEND);

            this->data_ptr->ordering += 1;
            msg = this->data_ptr->pub_interactions_queue.erase(msg);
        }
    }

    err = modality_attr_val_set_timestamp(&this->data_ptr->event_attrs[EID_IDX_TIMESTAMP].val, sim_time_ns);
    this->data_ptr->HandleClientError(err, "Failed to set event timestamp attribute value");
    err = modality_attr_val_set_timestamp(&this->data_ptr->event_attrs[EID_IDX_SIM_TIME].val, sim_time_ns);
    this->data_ptr->HandleClientError(err, "Failed to set event sim time attribute value");
    err = modality_attr_val_set_timestamp(&this->data_ptr->event_attrs[EID_IDX_WALL_CLOCK_TIME].val, wall_clock_time_ns);
    this->data_ptr->HandleClientError(err, "Failed to set event wall clock time attribute value");

    if(this->data_ptr->trace_pose)
    {
        if(auto maybe_pose = link.WorldPose(ecm))
        {
            auto pose = maybe_pose.value();
            err = modality_attr_val_set_string(&this->data_ptr->event_attrs[EID_IDX_NAME].val, EVENT_NAME_POSE);
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_X].val, pose.X());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Y].val, pose.Y());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Z].val, pose.Z());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_ROLL].val, pose.Roll());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_PITCH].val, pose.Pitch());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_YAW].val, pose.Yaw());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_ingest_client_event(
                    this->data_ptr->client,
                    this->data_ptr->ordering,
                    0,
                    &this->data_ptr->event_attrs[EID_IDX_NAME],
                    NUM_EVENT_ATTRS_POSE);
            this->data_ptr->HandleClientError(err, ERR_EVENT_SEND);

            this->data_ptr->ordering += 1;

            // Log once if static
            if(model_is_static)
            {
                this->data_ptr->trace_pose = false;
            }
        }
        else
        {
            gzwarn << "Link entity [" << this->data_ptr->entity << " : " << this->data_ptr->link_name << "] doesn't have a pose component" << std::endl;
        }
    }

    if(this->data_ptr->trace_linear_vel)
    {
        if(auto maybe_lin_vel = link.WorldLinearVelocity(ecm))
        {
            auto vel = maybe_lin_vel.value();
            err = modality_attr_val_set_string(&this->data_ptr->event_attrs[EID_IDX_NAME].val, EVENT_NAME_LINEAR_VEL);
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_X].val, vel.X());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Y].val, vel.Y());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Z].val, vel.Z());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_ingest_client_event(
                    this->data_ptr->client,
                    this->data_ptr->ordering,
                    0,
                    &this->data_ptr->event_attrs[EID_IDX_NAME],
                    NUM_EVENT_ATTRS_LINEAR_VEL);
            this->data_ptr->HandleClientError(err, ERR_EVENT_SEND);

            this->data_ptr->ordering += 1;

            // Log once if static
            if(model_is_static)
            {
                this->data_ptr->trace_linear_vel = false;
            }
        }
        else
        {
            gzwarn << "Link entity [" << this->data_ptr->entity << " : " << this->data_ptr->link_name << "] doesn't have a linear velocity component" << std::endl;
        }
    }

    if(this->data_ptr->trace_linear_accel)
    {
        if(auto maybe_lin_accel = link.WorldLinearAcceleration(ecm))
        {
            auto accel = maybe_lin_accel.value();
            err = modality_attr_val_set_string(&this->data_ptr->event_attrs[EID_IDX_NAME].val, EVENT_NAME_LINEAR_ACCEL);
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_X].val, accel.X());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Y].val, accel.Y());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);
            err = modality_attr_val_set_float(&this->data_ptr->event_attrs[EID_IDX_Z].val, accel.Z());
            this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

            err = modality_ingest_client_event(
                    this->data_ptr->client,
                    this->data_ptr->ordering,
                    0,
                    &this->data_ptr->event_attrs[EID_IDX_NAME],
                    NUM_EVENT_ATTRS_LINEAR_ACCEL);
            this->data_ptr->HandleClientError(err, ERR_EVENT_SEND);

            this->data_ptr->ordering += 1;

            // Log once if static
            if(model_is_static)
            {
                this->data_ptr->trace_linear_accel = false;
            }
        }
        else
        {
            gzwarn << "Link entity [" << this->data_ptr->entity << " : " << this->data_ptr->link_name << "] doesn't have a linear acceleration component" << std::endl;
        }
    }

    if(this->data_ptr->trace_contact_collision)
    {
        auto contacts = ecm.Component<gz::sim::components::ContactSensorData>(this->data_ptr->collision_entity);
        if(contacts != NULL)
        {
            if(contacts->Data().contact_size() > 0)
            {
                err = modality_attr_val_set_string(&this->data_ptr->event_attrs[EID_IDX_NAME].val, EVENT_NAME_CONTACT);
                this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);
            }

            for(const auto &contact : contacts->Data().contact())
            {
                if(contact.has_collision2())
                {
                    auto other_col_entity = contact.collision2();
                    auto other_name = gz::sim::scopedName(other_col_entity.id(), ecm, "::");

                    err = modality_attr_val_set_string(&this->data_ptr->event_attrs[EID_IDX_COLLISION_NAME].val, other_name.c_str());
                    this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

                    struct modality_big_int other_col_entity_id;
                    err = modality_big_int_set(&other_col_entity_id, other_col_entity.id(), 0);
                    this->data_ptr->HandleClientError(err, "Failed to set contact collision entity big int value");

                    err = modality_attr_val_set_big_int(&this->data_ptr->event_attrs[EID_IDX_COLLISION_ENTITY].val, &other_col_entity_id);
                    this->data_ptr->HandleClientError(err, ERR_EVENT_ATTR_VAL);

                    err = modality_ingest_client_event(
                            this->data_ptr->client,
                            this->data_ptr->ordering,
                            0,
                            &this->data_ptr->event_attrs[EID_IDX_COLLISION_NAME],
                            NUM_EVENT_ATTRS_CONTACT);
                    this->data_ptr->HandleClientError(err, ERR_EVENT_SEND);

                    this->data_ptr->ordering += 1;
                }
            }
        }
    }
}
