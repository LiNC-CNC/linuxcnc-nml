#ifndef LINUXCNC_NML_SHIM_H
#define LINUXCNC_NML_SHIM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LCN_NML_MAX_JOINTS 16
#define LCN_NML_MAX_AXES 9
#define LCN_NML_MAX_SPINDLES 8
#define LCN_NML_MAX_DIO 64
#define LCN_NML_MAX_AIO 64
#define LCN_NML_MAX_MISC_ERROR 64
#define LCN_NML_MAX_GCODES 17
#define LCN_NML_MAX_MCODES 10
#define LCN_NML_MAX_SETTINGS 5
#define LCN_NML_MAX_STATE_FIELDS 8
#define LCN_NML_MAX_STATE_FLOAT_FIELDS 5
#define LCN_NML_MAX_TOOLS 1001
#define LCN_NML_TEXT_LEN 256
#define LCN_NML_SOURCE_FILE_LEN 64

typedef struct LcnNmlPose {
    double x;
    double y;
    double z;
    double a;
    double b;
    double c;
    double u;
    double v;
    double w;
} LcnNmlPose;

typedef struct LcnNmlTool {
    int32_t toolno;
    int32_t pocketno;
    LcnNmlPose offset;
    double diameter;
    double frontangle;
    double backangle;
    int32_t orientation;
} LcnNmlTool;

typedef struct LcnNmlJoint {
    int32_t joint;
    int32_t joint_type;
    double units;
    double backlash;
    double min_position_limit;
    double max_position_limit;
    double max_ferror;
    double min_ferror;
    double ferror_current;
    double ferror_high_mark;
    double output;
    double input;
    double velocity;
    int32_t inpos;
    int32_t homing;
    int32_t homed;
    int32_t fault;
    int32_t enabled;
    int32_t min_soft_limit;
    int32_t max_soft_limit;
    int32_t min_hard_limit;
    int32_t max_hard_limit;
    int32_t override_limits;
} LcnNmlJoint;

typedef struct LcnNmlAxis {
    int32_t axis;
    double min_position_limit;
    double max_position_limit;
    double velocity;
} LcnNmlAxis;

typedef struct LcnNmlSpindle {
    double speed;
    double spindle_scale;
    double css_maximum;
    double css_factor;
    int32_t state;
    int32_t direction;
    int32_t brake;
    int32_t increasing;
    int32_t enabled;
    int32_t orient_state;
    int32_t orient_fault;
    int32_t spindle_override_enabled;
    int32_t homed;
} LcnNmlSpindle;

typedef struct LcnNmlStatus {
    int32_t valid;
    int32_t echo_serial_number;

    int32_t task_mode;
    int32_t task_state;
    int32_t exec_state;
    int32_t interp_state;
    int32_t call_level;
    int32_t current_line;
    int32_t read_line;
    int32_t motion_line;
    int32_t task_paused;
    int32_t interpreter_errcode;
    int32_t program_units;
    int32_t queued_mdi_commands;
    double delay_left;
    char file[LCN_NML_TEXT_LEN];
    char command[LCN_NML_TEXT_LEN];
    char ini_filename[LCN_NML_TEXT_LEN];

    double linear_units;
    double angular_units;
    double cycle_time;
    int32_t joints;
    int32_t spindles;
    int32_t axis_mask;
    int32_t motion_mode;
    int32_t enabled;
    int32_t inpos;
    int32_t queue;
    int32_t active_queue;
    int32_t queue_full;
    int32_t motion_id;
    int32_t paused;
    double feedrate;
    double rapidrate;
    double velocity;
    double acceleration;
    double max_velocity;
    double max_acceleration;
    int32_t probe_tripped;
    int32_t probing;
    int32_t probe_val;
    int32_t kinematics_type;
    int32_t motion_type;
    double distance_to_go;
    double current_vel;
    int32_t feed_override_enabled;
    int32_t adaptive_feed_enabled;
    int32_t feed_hold_enabled;

    LcnNmlPose position;
    LcnNmlPose actual_position;
    LcnNmlPose dtg;
    LcnNmlPose probed_position;

    LcnNmlPose g5x_offset;
    LcnNmlPose g92_offset;
    LcnNmlPose tool_offset;
    int32_t g5x_index;
    double rotation_xy;

    int32_t active_gcodes[LCN_NML_MAX_GCODES];
    int32_t active_mcodes[LCN_NML_MAX_MCODES];
    double active_settings[LCN_NML_MAX_SETTINGS];

    LcnNmlJoint joints_data[LCN_NML_MAX_JOINTS];
    LcnNmlAxis axes_data[LCN_NML_MAX_AXES];
    LcnNmlSpindle spindles_data[LCN_NML_MAX_SPINDLES];

    int32_t tool_in_spindle;
    int32_t pocket_prepped;
    int32_t tool_from_pocket;
    LcnNmlTool tool_table_current;

    int32_t mist;
    int32_t flood;
    int32_t estop;
    int32_t lube;
    int32_t lube_level;
    int32_t io_fault;
    int32_t io_reason;
    double io_cycle_time;

    int32_t num_extra_joints;
    int32_t on_soft_limit;
    int32_t external_offsets_applied;
    int32_t jogging_active;

    // RCS_STAT_MSG fields.
    int32_t command_type;
    int32_t rcs_status;
    int32_t rcs_state;
    int32_t rcs_line;
    int32_t rcs_source_line;
    char source_file[LCN_NML_SOURCE_FILE_LEN];

    // Remaining EMC_STAT fields.
    int32_t optional_stop;
    int32_t block_delete;
    int32_t input_timeout;
    int32_t debug;
    int32_t task_heartbeat;
    int32_t motion_heartbeat;
    uint32_t io_heartbeat;
    int32_t motion_debug;
    int32_t io_debug;

    int32_t synch_di[LCN_NML_MAX_DIO];
    int32_t synch_do[LCN_NML_MAX_DIO];
    double analog_input[LCN_NML_MAX_AIO];
    double analog_output[LCN_NML_MAX_AIO];
    int32_t misc_error[LCN_NML_MAX_MISC_ERROR];
    LcnNmlPose eoffset_pose;

    float tag_fields_float[LCN_NML_MAX_STATE_FLOAT_FIELDS];
    int32_t tag_fields[LCN_NML_MAX_STATE_FIELDS];
    uint64_t tag_packed_flags;
    uint64_t tag_flags;
} LcnNmlStatus;

typedef struct LcnNmlHandle LcnNmlHandle;

LcnNmlHandle *lcn_nml_open(const char *nmlfile, char *error, size_t error_len);
void lcn_nml_close(LcnNmlHandle *handle);

const char *lcn_nml_last_error(const LcnNmlHandle *handle);
uint32_t lcn_nml_status_size(void);
uint32_t lcn_nml_stat_size(void);
const char *lcn_nml_version(void);

int32_t lcn_nml_poll_status(LcnNmlHandle *handle, LcnNmlStatus *out);
int32_t lcn_nml_poll_error(LcnNmlHandle *handle, int32_t *kind, char *text, size_t text_len);
int32_t lcn_nml_wait_complete(LcnNmlHandle *handle, double timeout_seconds);
int32_t lcn_nml_last_serial(const LcnNmlHandle *handle);
int32_t lcn_nml_tool_table(LcnNmlHandle *handle, LcnNmlTool *out, size_t capacity,
                           size_t *count);

int32_t lcn_nml_set_debug(LcnNmlHandle *handle, int32_t debug);
int32_t lcn_nml_teleop_enable(LcnNmlHandle *handle, int32_t enable);
int32_t lcn_nml_set_traj_mode(LcnNmlHandle *handle, int32_t mode);
int32_t lcn_nml_set_mode(LcnNmlHandle *handle, int32_t mode);
int32_t lcn_nml_set_state(LcnNmlHandle *handle, int32_t state);
int32_t lcn_nml_mdi(LcnNmlHandle *handle, const char *block);
int32_t lcn_nml_program_open(LcnNmlHandle *handle, const char *path);
int32_t lcn_nml_auto_run(LcnNmlHandle *handle, int32_t line);
int32_t lcn_nml_auto_pause(LcnNmlHandle *handle);
int32_t lcn_nml_auto_resume(LcnNmlHandle *handle);
int32_t lcn_nml_auto_step(LcnNmlHandle *handle);
int32_t lcn_nml_auto_reverse(LcnNmlHandle *handle);
int32_t lcn_nml_auto_forward(LcnNmlHandle *handle);
int32_t lcn_nml_reset_interpreter(LcnNmlHandle *handle);
int32_t lcn_nml_abort(LcnNmlHandle *handle);
int32_t lcn_nml_task_plan_synch(LcnNmlHandle *handle);

int32_t lcn_nml_feedrate(LcnNmlHandle *handle, double scale);
int32_t lcn_nml_rapidrate(LcnNmlHandle *handle, double scale);
int32_t lcn_nml_maxvel(LcnNmlHandle *handle, double velocity);
int32_t lcn_nml_spindle_override(LcnNmlHandle *handle, double scale, int32_t spindle);
int32_t lcn_nml_spindle_on(LcnNmlHandle *handle, double speed, int32_t spindle,
                           int32_t wait_for_speed);
int32_t lcn_nml_spindle_off(LcnNmlHandle *handle, int32_t spindle);
int32_t lcn_nml_spindle_increase(LcnNmlHandle *handle, int32_t spindle);
int32_t lcn_nml_spindle_decrease(LcnNmlHandle *handle, int32_t spindle);
int32_t lcn_nml_spindle_constant(LcnNmlHandle *handle, int32_t spindle);
int32_t lcn_nml_spindle_brake(LcnNmlHandle *handle, int32_t spindle, int32_t engage);

int32_t lcn_nml_tool_offset(LcnNmlHandle *handle, int32_t toolno, double z, double x,
                            double diameter, double frontangle, double backangle,
                            int32_t orientation);
int32_t lcn_nml_mist(LcnNmlHandle *handle, int32_t on);
int32_t lcn_nml_flood(LcnNmlHandle *handle, int32_t on);
int32_t lcn_nml_override_limits(LcnNmlHandle *handle);
int32_t lcn_nml_home(LcnNmlHandle *handle, int32_t joint);
int32_t lcn_nml_unhome(LcnNmlHandle *handle, int32_t joint);
int32_t lcn_nml_jog_stop(LcnNmlHandle *handle, int32_t joint_mode, int32_t index);
int32_t lcn_nml_jog_continuous(LcnNmlHandle *handle, int32_t joint_mode, int32_t index,
                               double velocity);
int32_t lcn_nml_jog_increment(LcnNmlHandle *handle, int32_t joint_mode, int32_t index,
                              double velocity, double increment);

int32_t lcn_nml_set_optional_stop(LcnNmlHandle *handle, int32_t on);
int32_t lcn_nml_set_block_delete(LcnNmlHandle *handle, int32_t on);
int32_t lcn_nml_set_min_limit(LcnNmlHandle *handle, int32_t joint, double limit);
int32_t lcn_nml_set_max_limit(LcnNmlHandle *handle, int32_t joint, double limit);
int32_t lcn_nml_set_feed_override(LcnNmlHandle *handle, int32_t on);
int32_t lcn_nml_set_spindle_override(LcnNmlHandle *handle, int32_t on, int32_t spindle);
int32_t lcn_nml_set_feed_hold(LcnNmlHandle *handle, int32_t on);
int32_t lcn_nml_set_adaptive_feed(LcnNmlHandle *handle, int32_t on);
int32_t lcn_nml_set_digital_output(LcnNmlHandle *handle, int32_t index, int32_t on);
int32_t lcn_nml_set_analog_output(LcnNmlHandle *handle, int32_t index, double value);

int32_t lcn_nml_error_msg(LcnNmlHandle *handle, const char *text);
int32_t lcn_nml_text_msg(LcnNmlHandle *handle, const char *text);
int32_t lcn_nml_display_msg(LcnNmlHandle *handle, const char *text);
int32_t lcn_nml_load_tool_table(LcnNmlHandle *handle, const char *file);

#ifdef __cplusplus
}
#endif

#endif
