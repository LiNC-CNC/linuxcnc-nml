#include "lcn_nml.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <exception>
#include <new>
#include <string>
#include <thread>

#include "cmd_msg.hh"
#include "emc.hh"
#include "emc_nml.hh"
#include "emccfg.h"
#include "emcglb.h"
#include "interp_base.hh"
#include "nml.hh"
#include "nml_oi.hh"
#include "state_tag.h"
#include "stat_msg.hh"

// The installed LinuxCNC headers do not expose tooldata.hh. These are the
// C++ symbols exported by libtooldata.
int tooldata_last_index_get(void);
int tooldata_get(CANON_TOOL_TABLE *data, int index);
int tool_mmap_user(void);

struct LcnNmlHandle {
    RCS_STAT_CHANNEL *stat = nullptr;
    RCS_CMD_CHANNEL *cmd = nullptr;
    NML *error = nullptr;
    EMC_STAT status;
    bool has_status = false;
    bool tool_mmap_tried = false;
    int serial = 0;
    std::string last_error;

    ~LcnNmlHandle() {
        delete error;
        delete cmd;
        delete stat;
    }
};

namespace {

constexpr double kCommandTimeout = 5.0;
constexpr double kPollInterval = 0.01;

void set_error(LcnNmlHandle *handle, const char *format, ...) {
    if (handle == nullptr) {
        return;
    }
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    buffer[sizeof(buffer) - 1] = '\0';
    handle->last_error = buffer;
}

void clear_error(LcnNmlHandle *handle) {
    if (handle != nullptr) {
        handle->last_error.clear();
    }
}

void copy_text(char *dst, size_t len, const char *src) {
    if (dst == nullptr || len == 0) {
        return;
    }
    if (src == nullptr) {
        src = "";
    }
    // Copy at most len-1 bytes and always terminate. Using memcpy with an
    // explicit bound avoids GCC's -Wstringop-truncation false positive for
    // strncpy when the source is exactly as long as the destination.
    size_t n = std::strlen(src);
    if (n >= len) {
        n = len - 1;
    }
    std::memmove(dst, src, n);
    dst[n] = '\0';
}

void copy_error(char *dst, size_t len, const char *src) {
    copy_text(dst, len, src == nullptr ? "" : src);
}

void pose_to_c(LcnNmlPose *out, const EmcPose &pose) {
    out->x = pose.tran.x;
    out->y = pose.tran.y;
    out->z = pose.tran.z;
    out->a = pose.a;
    out->b = pose.b;
    out->c = pose.c;
    out->u = pose.u;
    out->v = pose.v;
    out->w = pose.w;
}

void tool_to_c(LcnNmlTool *out, const CANON_TOOL_TABLE &tool) {
    out->toolno = tool.toolno;
    out->pocketno = tool.pocketno;
    pose_to_c(&out->offset, tool.offset);
    out->diameter = tool.diameter;
    out->frontangle = tool.frontangle;
    out->backangle = tool.backangle;
    out->orientation = tool.orientation;
}

int error_format(NMLTYPE type, void *buffer, CMS *cms) {
    // emcError carries both EMC_OPERATOR_* and generic NML_* messages.
    if (nmlErrorFormat(type, buffer, cms) == 1) {
        return 1;
    }
    return emcFormat(type, buffer, cms);
}

int refresh_status(LcnNmlHandle *handle) {
    if (handle == nullptr || handle->stat == nullptr) {
        set_error(handle, "status channel is not open");
        return -1;
    }
    if (!handle->stat->valid()) {
        set_error(handle, "status channel is invalid");
        return -1;
    }

    NMLTYPE type = handle->stat->peek();
    if (type == -1) {
        set_error(handle, "status read failed");
        return -1;
    }
    if (type == EMC_STAT_TYPE) {
        EMC_STAT *status = static_cast<EMC_STAT *>(handle->stat->get_address());
        if (status == nullptr) {
            set_error(handle, "status channel returned no data");
            return -1;
        }
        std::memcpy(reinterpret_cast<void *>(&handle->status),
                    reinterpret_cast<const void *>(status), sizeof(EMC_STAT));
        handle->has_status = true;
    }
    return 0;
}

void fill_status(const LcnNmlHandle *handle, LcnNmlStatus *out) {
    std::memset(out, 0, sizeof(*out));
    if (!handle->has_status) {
        return;
    }

    const EMC_STAT &s = handle->status;
    out->valid = 1;
    out->echo_serial_number = s.echo_serial_number;

    out->task_mode = static_cast<int32_t>(s.task.mode);
    out->task_state = static_cast<int32_t>(s.task.state);
    out->exec_state = static_cast<int32_t>(s.task.execState);
    out->interp_state = static_cast<int32_t>(s.task.interpState);
    out->call_level = s.task.callLevel;
    out->current_line = s.task.currentLine;
    out->read_line = s.task.readLine;
    out->motion_line = s.task.motionLine;
    out->task_paused = s.task.task_paused;
    out->interpreter_errcode = s.task.interpreter_errcode;
    out->program_units = static_cast<int32_t>(s.task.programUnits);
    out->queued_mdi_commands = s.task.queuedMDIcommands;
    out->delay_left = s.task.delayLeft;
    copy_text(out->file, sizeof(out->file), s.task.file);
    copy_text(out->command, sizeof(out->command), s.task.command);
    copy_text(out->ini_filename, sizeof(out->ini_filename), s.task.ini_filename);
    out->optional_stop = s.task.optional_stop_state ? 1 : 0;
    out->block_delete = s.task.block_delete_state ? 1 : 0;
    out->input_timeout = s.task.input_timeout ? 1 : 0;
    out->task_heartbeat = static_cast<int32_t>(s.task.heartbeat);

    out->linear_units = s.motion.traj.linearUnits;
    out->angular_units = s.motion.traj.angularUnits;
    out->cycle_time = s.motion.traj.cycleTime;
    out->joints = s.motion.traj.joints;
    out->spindles = s.motion.traj.spindles;
    out->axis_mask = s.motion.traj.axis_mask;
    out->motion_mode = static_cast<int32_t>(s.motion.traj.mode);
    out->enabled = s.motion.traj.enabled ? 1 : 0;
    out->inpos = s.motion.traj.inpos ? 1 : 0;
    out->queue = s.motion.traj.queue;
    out->active_queue = s.motion.traj.activeQueue;
    out->queue_full = s.motion.traj.queueFull ? 1 : 0;
    out->motion_id = s.motion.traj.id;
    out->paused = s.motion.traj.paused ? 1 : 0;
    out->feedrate = s.motion.traj.scale;
    out->rapidrate = s.motion.traj.rapid_scale;
    out->velocity = s.motion.traj.velocity;
    out->acceleration = s.motion.traj.acceleration;
    out->max_velocity = s.motion.traj.maxVelocity;
    out->max_acceleration = s.motion.traj.maxAcceleration;
    out->probe_tripped = s.motion.traj.probe_tripped ? 1 : 0;
    out->probing = s.motion.traj.probing ? 1 : 0;
    out->probe_val = s.motion.traj.probeval;
    out->kinematics_type = s.motion.traj.kinematics_type;
    out->motion_type = s.motion.traj.motion_type;
    out->distance_to_go = s.motion.traj.distance_to_go;
    out->current_vel = s.motion.traj.current_vel;
    out->feed_override_enabled = s.motion.traj.feed_override_enabled ? 1 : 0;
    out->adaptive_feed_enabled = s.motion.traj.adaptive_feed_enabled ? 1 : 0;
    out->feed_hold_enabled = s.motion.traj.feed_hold_enabled ? 1 : 0;
    out->motion_heartbeat = static_cast<int32_t>(s.motion.heartbeat);
    out->motion_debug = s.motion.debug;

    pose_to_c(&out->position, s.motion.traj.position);
    pose_to_c(&out->actual_position, s.motion.traj.actualPosition);
    pose_to_c(&out->dtg, s.motion.traj.dtg);
    pose_to_c(&out->probed_position, s.motion.traj.probedPosition);

    pose_to_c(&out->g5x_offset, s.task.g5x_offset);
    pose_to_c(&out->g92_offset, s.task.g92_offset);
    pose_to_c(&out->tool_offset, s.task.toolOffset);
    out->g5x_index = s.task.g5x_index;
    out->rotation_xy = s.task.rotation_xy;

    out->command_type = s.command_type;
    out->rcs_status = s.status;
    out->rcs_state = s.state;
    out->rcs_line = s.line;
    out->rcs_source_line = s.source_line;
    copy_text(out->source_file, sizeof(out->source_file), s.source_file);
    out->debug = s.debug;

    for (int i = 0; i < LCN_NML_MAX_GCODES; ++i) {
        out->active_gcodes[i] = s.task.activeGCodes[i];
    }
    for (int i = 0; i < LCN_NML_MAX_MCODES; ++i) {
        out->active_mcodes[i] = s.task.activeMCodes[i];
    }
    for (int i = 0; i < LCN_NML_MAX_SETTINGS; ++i) {
        out->active_settings[i] = s.task.activeSettings[i];
    }

    for (int i = 0; i < LCN_NML_MAX_JOINTS; ++i) {
        const EMC_JOINT_STAT &joint = s.motion.joint[i];
        LcnNmlJoint &out_joint = out->joints_data[i];
        out_joint.joint = joint.joint;
        out_joint.joint_type = joint.jointType;
        out_joint.units = joint.units;
        out_joint.backlash = joint.backlash;
        out_joint.min_position_limit = joint.minPositionLimit;
        out_joint.max_position_limit = joint.maxPositionLimit;
        out_joint.max_ferror = joint.maxFerror;
        out_joint.min_ferror = joint.minFerror;
        out_joint.ferror_current = joint.ferrorCurrent;
        out_joint.ferror_high_mark = joint.ferrorHighMark;
        out_joint.output = joint.output;
        out_joint.input = joint.input;
        out_joint.velocity = joint.velocity;
        out_joint.inpos = joint.inpos;
        out_joint.homing = joint.homing;
        out_joint.homed = joint.homed;
        out_joint.fault = joint.fault;
        out_joint.enabled = joint.enabled;
        out_joint.min_soft_limit = joint.minSoftLimit;
        out_joint.max_soft_limit = joint.maxSoftLimit;
        out_joint.min_hard_limit = joint.minHardLimit;
        out_joint.max_hard_limit = joint.maxHardLimit;
        out_joint.override_limits = joint.overrideLimits;
    }

    for (int i = 0; i < LCN_NML_MAX_AXES; ++i) {
        const EMC_AXIS_STAT &axis = s.motion.axis[i];
        LcnNmlAxis &out_axis = out->axes_data[i];
        out_axis.axis = axis.axis;
        out_axis.min_position_limit = axis.minPositionLimit;
        out_axis.max_position_limit = axis.maxPositionLimit;
        out_axis.velocity = axis.velocity;
    }

    for (int i = 0; i < LCN_NML_MAX_SPINDLES; ++i) {
        const EMC_SPINDLE_STAT &spindle = s.motion.spindle[i];
        LcnNmlSpindle &out_spindle = out->spindles_data[i];
        out_spindle.speed = spindle.speed;
        out_spindle.spindle_scale = spindle.spindle_scale;
        out_spindle.css_maximum = spindle.css_maximum;
        out_spindle.css_factor = spindle.css_factor;
        out_spindle.state = spindle.state;
        out_spindle.direction = spindle.direction;
        out_spindle.brake = spindle.brake;
        out_spindle.increasing = spindle.increasing;
        out_spindle.enabled = spindle.enabled;
        out_spindle.orient_state = spindle.orient_state;
        out_spindle.orient_fault = spindle.orient_fault;
        out_spindle.spindle_override_enabled = spindle.spindle_override_enabled ? 1 : 0;
        out_spindle.homed = spindle.homed ? 1 : 0;
    }

    out->tool_in_spindle = s.io.tool.toolInSpindle;
    out->pocket_prepped = s.io.tool.pocketPrepped;
    out->tool_from_pocket = s.io.tool.toolFromPocket;
    tool_to_c(&out->tool_table_current, s.io.tool.toolTableCurrent);
    out->mist = s.io.coolant.mist;
    out->flood = s.io.coolant.flood;
    out->estop = s.io.aux.estop;
    out->lube = s.io.lube.on;
    out->lube_level = s.io.lube.level;
    out->io_fault = s.io.fault;
    out->io_reason = s.io.reason;
    out->io_cycle_time = s.io.cycleTime;
    out->io_heartbeat = s.io.heartbeat;
    out->io_debug = s.io.debug;

    for (int i = 0; i < LCN_NML_MAX_DIO; ++i) {
        out->synch_di[i] = s.motion.synch_di[i];
        out->synch_do[i] = s.motion.synch_do[i];
    }
    for (int i = 0; i < LCN_NML_MAX_AIO; ++i) {
        out->analog_input[i] = s.motion.analog_input[i];
        out->analog_output[i] = s.motion.analog_output[i];
    }
    for (int i = 0; i < LCN_NML_MAX_MISC_ERROR; ++i) {
        out->misc_error[i] = s.motion.misc_error[i];
    }
    pose_to_c(&out->eoffset_pose, s.motion.eoffset_pose);

    for (int i = 0; i < LCN_NML_MAX_STATE_FLOAT_FIELDS; ++i) {
        out->tag_fields_float[i] = s.motion.traj.tag.fields_float[i];
    }
    for (int i = 0; i < LCN_NML_MAX_STATE_FIELDS; ++i) {
        out->tag_fields[i] = s.motion.traj.tag.fields[i];
    }
    out->tag_packed_flags = s.motion.traj.tag.packed_flags;
    out->tag_flags = s.motion.traj.tag.flags.to_ullong();

    out->num_extra_joints = s.motion.numExtraJoints;
    out->on_soft_limit = s.motion.on_soft_limit;
    out->external_offsets_applied = s.motion.external_offsets_applied;
    out->jogging_active = s.motion.jogging_active ? 1 : 0;
}

int wait_for_echo(LcnNmlHandle *handle, int serial, double timeout_seconds) {
    if (timeout_seconds < 0.0) {
        timeout_seconds = kCommandTimeout;
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_seconds);

    while (std::chrono::steady_clock::now() < deadline) {
        if (refresh_status(handle) != 0) {
            return -1;
        }
        if (handle->has_status && handle->status.echo_serial_number - serial >= 0) {
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(kPollInterval));
    }

    set_error(handle, "timed out waiting for command %d to be accepted", serial);
    return 0;
}

int wait_for_complete(LcnNmlHandle *handle, int serial, double timeout_seconds) {
    if (timeout_seconds < 0.0) {
        timeout_seconds = kCommandTimeout;
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_seconds);

    while (std::chrono::steady_clock::now() < deadline) {
        if (refresh_status(handle) != 0) {
            return -1;
        }
        if (handle->has_status) {
            const int diff = handle->status.echo_serial_number - serial;
            if (diff > 0) {
                return 1;
            }
            if (diff == 0 && (handle->status.status == RCS_DONE ||
                              handle->status.status == RCS_ERROR)) {
                return 1;
            }
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(kPollInterval));
    }

    set_error(handle, "timed out waiting for command %d to complete", serial);
    return 0;
}

template <typename Message>
int send_command(LcnNmlHandle *handle, Message &message) {
    if (handle == nullptr || handle->cmd == nullptr) {
        set_error(handle, "command channel is not open");
        return -1;
    }
    if (!handle->cmd->valid()) {
        set_error(handle, "command channel is invalid");
        return -1;
    }

    clear_error(handle);
    if (handle->cmd->write(&message) != 0) {
        set_error(handle, "command write failed");
        return -1;
    }
    handle->serial = message.serial_number;
    const int result = wait_for_echo(handle, handle->serial, kCommandTimeout);
    return result == 1 ? 0 : -1;
}

bool copy_command_text(char *dst, size_t len, const char *src, LcnNmlHandle *handle,
                       const char *what) {
    if (src == nullptr) {
        set_error(handle, "%s is null", what);
        return false;
    }
    const size_t source_len = std::strlen(src);
    if (source_len >= len) {
        set_error(handle, "%s is too long (max %zu bytes)", what, len - 1);
        return false;
    }
    std::memcpy(dst, src, source_len + 1);
    return true;
}

}  // namespace

extern "C" LcnNmlHandle *lcn_nml_open(const char *nmlfile, char *error, size_t error_len) {
    const char *path = nmlfile;
    if (path == nullptr || path[0] == '\0') {
        path = DEFAULT_EMC_NMLFILE;
    }
    if (path == nullptr || path[0] == '\0') {
        copy_error(error, error_len, "no NML configuration file");
        return nullptr;
    }

    LcnNmlHandle *handle = nullptr;
    try {
        handle = new LcnNmlHandle();

        handle->stat = new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "xemc", path);
        if (!handle->stat->valid()) {
            set_error(handle, "cannot open emcStatus from %s", path);
            copy_error(error, error_len, handle->last_error.c_str());
            delete handle;
            return nullptr;
        }

        handle->cmd = new RCS_CMD_CHANNEL(emcFormat, "emcCommand", "xemc", path);
        if (!handle->cmd->valid()) {
            set_error(handle, "cannot open emcCommand from %s", path);
            copy_error(error, error_len, handle->last_error.c_str());
            delete handle;
            return nullptr;
        }

        handle->error = new NML(error_format, "emcError", "xemc", path);
        if (!handle->error->valid()) {
            set_error(handle, "cannot open emcError from %s", path);
            copy_error(error, error_len, handle->last_error.c_str());
            delete handle;
            return nullptr;
        }
    } catch (const std::exception &exception) {
        copy_error(error, error_len, exception.what());
        delete handle;
        return nullptr;
    } catch (...) {
        copy_error(error, error_len, "unknown exception opening LinuxCNC NML channels");
        delete handle;
        return nullptr;
    }

    copy_error(error, error_len, "");
    return handle;
}

extern "C" void lcn_nml_close(LcnNmlHandle *handle) {
    delete handle;
}

extern "C" const char *lcn_nml_last_error(const LcnNmlHandle *handle) {
    if (handle == nullptr) {
        return "";
    }
    return handle->last_error.c_str();
}

extern "C" uint32_t lcn_nml_status_size(void) {
    return static_cast<uint32_t>(sizeof(LcnNmlStatus));
}

extern "C" uint32_t lcn_nml_stat_size(void) {
    return static_cast<uint32_t>(sizeof(EMC_STAT));
}

extern "C" const char *lcn_nml_version(void) {
#ifdef LCN_NML_VERSION
    return LCN_NML_VERSION;
#else
    return "unknown";
#endif
}

extern "C" int32_t lcn_nml_poll_status(LcnNmlHandle *handle, LcnNmlStatus *out) {
    if (out == nullptr) {
        set_error(handle, "status output is null");
        return -1;
    }
    clear_error(handle);
    if (refresh_status(handle) != 0) {
        return -1;
    }
    fill_status(handle, out);
    return 0;
}

extern "C" int32_t lcn_nml_poll_error(LcnNmlHandle *handle, int32_t *kind, char *text,
                                      size_t text_len) {
    if (handle == nullptr || handle->error == nullptr) {
        set_error(handle, "error channel is not open");
        return -1;
    }
    if (kind == nullptr || text == nullptr || text_len == 0) {
        set_error(handle, "invalid error output buffer");
        return -1;
    }
    clear_error(handle);
    if (!handle->error->valid()) {
        set_error(handle, "error channel is invalid");
        return -1;
    }

    const NMLTYPE type = handle->error->read();
    if (type == 0) {
        return 0;
    }
    if (type == static_cast<NMLTYPE>(-1)) {
        set_error(handle, "error channel read failed");
        return -1;
    }

    *kind = static_cast<int32_t>(type);
    void *address = handle->error->get_address();
    if (address == nullptr) {
        set_error(handle, "error channel returned no message");
        return -1;
    }
    switch (type) {
    case EMC_OPERATOR_ERROR_TYPE:
        copy_text(text, text_len,
                  static_cast<EMC_OPERATOR_ERROR *>(address)->error);
        break;
    case EMC_OPERATOR_TEXT_TYPE:
        copy_text(text, text_len,
                  static_cast<EMC_OPERATOR_TEXT *>(address)->text);
        break;
    case EMC_OPERATOR_DISPLAY_TYPE:
        copy_text(text, text_len,
                  static_cast<EMC_OPERATOR_DISPLAY *>(address)->display);
        break;
    case NML_ERROR_TYPE:
        copy_text(text, text_len, static_cast<NML_ERROR *>(address)->error);
        break;
    case NML_TEXT_TYPE:
        copy_text(text, text_len, static_cast<NML_TEXT *>(address)->text);
        break;
    case NML_DISPLAY_TYPE:
        copy_text(text, text_len, static_cast<NML_DISPLAY *>(address)->display);
        break;
    default:
        std::snprintf(text, text_len, "unrecognized error %d", static_cast<int>(type));
        break;
    }
    return 1;
}

extern "C" int32_t lcn_nml_wait_complete(LcnNmlHandle *handle, double timeout_seconds) {
    if (handle == nullptr || handle->serial <= 0) {
        set_error(handle, "no command is pending");
        return -1;
    }
    clear_error(handle);
    const int result = wait_for_complete(handle, handle->serial, timeout_seconds);
    if (result < 0) {
        return -1;
    }
    return result;
}

extern "C" int32_t lcn_nml_last_serial(const LcnNmlHandle *handle) {
    return handle == nullptr ? 0 : handle->serial;
}

extern "C" int32_t lcn_nml_tool_table(LcnNmlHandle *handle, LcnNmlTool *out, size_t capacity,
                                      size_t *count) {
    if (handle == nullptr || count == nullptr) {
        set_error(handle, "invalid tool table output");
        return -1;
    }
    clear_error(handle);
    if (!handle->tool_mmap_tried) {
        handle->tool_mmap_tried = true;
        tool_mmap_user();
    }

    const int last = tooldata_last_index_get() + 1;
    const size_t total = last > 0 ? static_cast<size_t>(last) : 0;
    *count = total;
    if (out == nullptr || capacity == 0) {
        return 0;
    }
    if (total > capacity) {
        set_error(handle, "tool table capacity %zu is smaller than %zu", capacity, total);
        return -1;
    }
    for (size_t i = 0; i < total; ++i) {
        CANON_TOOL_TABLE tool;
        std::memset(&tool, 0, sizeof(tool));
        if (tooldata_get(&tool, static_cast<int>(i)) != 0) {
            set_error(handle, "cannot read tool table entry %zu", i);
            return -1;
        }
        tool_to_c(&out[i], tool);
    }
    return 0;
}

extern "C" int32_t lcn_nml_set_debug(LcnNmlHandle *handle, int32_t debug) {
    EMC_SET_DEBUG message;
    message.debug = debug;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_teleop_enable(LcnNmlHandle *handle, int32_t enable) {
    EMC_TRAJ_SET_TELEOP_ENABLE message;
    message.enable = enable;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_traj_mode(LcnNmlHandle *handle, int32_t mode) {
    EMC_TRAJ_SET_MODE message;
    message.mode = static_cast<EMC_TRAJ_MODE_ENUM>(mode);
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_feedrate(LcnNmlHandle *handle, double scale) {
    EMC_TRAJ_SET_SCALE message;
    message.scale = scale;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_rapidrate(LcnNmlHandle *handle, double scale) {
    EMC_TRAJ_SET_RAPID_SCALE message;
    message.scale = scale;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_maxvel(LcnNmlHandle *handle, double velocity) {
    EMC_TRAJ_SET_MAX_VELOCITY message;
    message.velocity = velocity;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_spindle_override(LcnNmlHandle *handle, double scale,
                                            int32_t spindle) {
    EMC_TRAJ_SET_SPINDLE_SCALE message;
    message.spindle = spindle;
    message.scale = scale;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_spindle_on(LcnNmlHandle *handle, double speed, int32_t spindle,
                                      int32_t wait_for_speed) {
    EMC_SPINDLE_ON message;
    message.spindle = spindle;
    message.speed = speed;
    message.wait_for_spindle_at_speed = wait_for_speed;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_spindle_off(LcnNmlHandle *handle, int32_t spindle) {
    EMC_SPINDLE_OFF message;
    message.spindle = spindle;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_spindle_increase(LcnNmlHandle *handle, int32_t spindle) {
    EMC_SPINDLE_INCREASE message;
    message.spindle = spindle;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_spindle_decrease(LcnNmlHandle *handle, int32_t spindle) {
    EMC_SPINDLE_DECREASE message;
    message.spindle = spindle;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_spindle_constant(LcnNmlHandle *handle, int32_t spindle) {
    EMC_SPINDLE_CONSTANT message;
    message.spindle = spindle;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_spindle_brake(LcnNmlHandle *handle, int32_t spindle,
                                         int32_t engage) {
    if (engage != 0) {
        EMC_SPINDLE_BRAKE_ENGAGE message;
        message.spindle = spindle;
        return send_command(handle, message);
    }
    EMC_SPINDLE_BRAKE_RELEASE message;
    message.spindle = spindle;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_tool_offset(LcnNmlHandle *handle, int32_t toolno, double z, double x,
                                       double diameter, double frontangle, double backangle,
                                       int32_t orientation) {
    EMC_TOOL_SET_OFFSET message;
    message.toolno = toolno;
    message.offset.tran.z = z;
    message.offset.tran.x = x;
    message.diameter = diameter;
    message.frontangle = frontangle;
    message.backangle = backangle;
    message.orientation = orientation;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_mist(LcnNmlHandle *handle, int32_t on) {
    if (on != 0) {
        EMC_COOLANT_MIST_ON message;
        return send_command(handle, message);
    }
    EMC_COOLANT_MIST_OFF message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_flood(LcnNmlHandle *handle, int32_t on) {
    if (on != 0) {
        EMC_COOLANT_FLOOD_ON message;
        return send_command(handle, message);
    }
    EMC_COOLANT_FLOOD_OFF message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_abort(LcnNmlHandle *handle) {
    EMC_TASK_ABORT message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_task_plan_synch(LcnNmlHandle *handle) {
    EMC_TASK_PLAN_SYNCH message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_home(LcnNmlHandle *handle, int32_t joint) {
    EMC_JOINT_HOME message;
    message.joint = joint;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_unhome(LcnNmlHandle *handle, int32_t joint) {
    EMC_JOINT_UNHOME message;
    message.joint = joint;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_jog_stop(LcnNmlHandle *handle, int32_t joint_mode,
                                    int32_t index) {
    EMC_JOG_STOP message;
    message.joint_or_axis = index;
    message.jjogmode = joint_mode;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_jog_continuous(LcnNmlHandle *handle, int32_t joint_mode,
                                          int32_t index, double velocity) {
    EMC_JOG_CONT message;
    message.joint_or_axis = index;
    message.vel = velocity;
    message.jjogmode = joint_mode;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_jog_increment(LcnNmlHandle *handle, int32_t joint_mode,
                                         int32_t index, double velocity,
                                         double increment) {
    EMC_JOG_INCR message;
    message.joint_or_axis = index;
    message.vel = velocity;
    message.incr = increment;
    message.jjogmode = joint_mode;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_auto_pause(LcnNmlHandle *handle) {
    EMC_TASK_PLAN_PAUSE message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_auto_resume(LcnNmlHandle *handle) {
    EMC_TASK_PLAN_RESUME message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_auto_step(LcnNmlHandle *handle) {
    EMC_TASK_PLAN_STEP message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_auto_reverse(LcnNmlHandle *handle) {
    EMC_TASK_PLAN_REVERSE message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_auto_forward(LcnNmlHandle *handle) {
    EMC_TASK_PLAN_FORWARD message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_optional_stop(LcnNmlHandle *handle, int32_t on) {
    EMC_TASK_PLAN_SET_OPTIONAL_STOP message;
    message.state = on != 0;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_block_delete(LcnNmlHandle *handle, int32_t on) {
    EMC_TASK_PLAN_SET_BLOCK_DELETE message;
    message.state = on != 0;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_min_limit(LcnNmlHandle *handle, int32_t joint, double limit) {
    EMC_JOINT_SET_MIN_POSITION_LIMIT message;
    message.joint = joint;
    message.limit = limit;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_max_limit(LcnNmlHandle *handle, int32_t joint, double limit) {
    EMC_JOINT_SET_MAX_POSITION_LIMIT message;
    message.joint = joint;
    message.limit = limit;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_feed_override(LcnNmlHandle *handle, int32_t on) {
    EMC_TRAJ_SET_FO_ENABLE message;
    message.mode = on != 0 ? 1 : 0;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_spindle_override(LcnNmlHandle *handle, int32_t on,
                                                int32_t spindle) {
    EMC_TRAJ_SET_SO_ENABLE message;
    message.spindle = spindle;
    message.mode = on != 0 ? 1 : 0;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_feed_hold(LcnNmlHandle *handle, int32_t on) {
    EMC_TRAJ_SET_FH_ENABLE message;
    message.mode = on != 0 ? 1 : 0;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_adaptive_feed(LcnNmlHandle *handle, int32_t on) {
    EMC_MOTION_ADAPTIVE message;
    message.status = on != 0 ? 1 : 0;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_digital_output(LcnNmlHandle *handle, int32_t index,
                                               int32_t on) {
    EMC_MOTION_SET_DOUT message;
    message.index = static_cast<unsigned char>(index);
    message.start = on != 0 ? 1 : 0;
    message.now = 1;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_analog_output(LcnNmlHandle *handle, int32_t index,
                                              double value) {
    EMC_MOTION_SET_AOUT message;
    message.index = static_cast<unsigned char>(index);
    message.start = value;
    message.now = 1;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_error_msg(LcnNmlHandle *handle, const char *text) {
    EMC_OPERATOR_ERROR message;
    message.id = 0;
    if (!copy_command_text(message.error, sizeof(message.error), text, handle,
                           "operator error")) {
        return -1;
    }
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_text_msg(LcnNmlHandle *handle, const char *text) {
    EMC_OPERATOR_TEXT message;
    message.id = 0;
    if (!copy_command_text(message.text, sizeof(message.text), text, handle,
                           "operator text")) {
        return -1;
    }
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_display_msg(LcnNmlHandle *handle, const char *text) {
    EMC_OPERATOR_DISPLAY message;
    message.id = 0;
    if (!copy_command_text(message.display, sizeof(message.display), text, handle,
                           "operator display")) {
        return -1;
    }
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_mode(LcnNmlHandle *handle, int32_t mode) {
    EMC_TASK_SET_MODE message;
    switch (mode) {
    case EMC_TASK_MODE_MANUAL:
    case EMC_TASK_MODE_AUTO:
    case EMC_TASK_MODE_MDI:
        break;
    default:
        set_error(handle, "invalid task mode %d", mode);
        return -1;
    }
    message.mode = static_cast<EMC_TASK_MODE_ENUM>(mode);
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_set_state(LcnNmlHandle *handle, int32_t state) {
    EMC_TASK_SET_STATE message;
    switch (state) {
    case EMC_TASK_STATE_ESTOP:
    case EMC_TASK_STATE_ESTOP_RESET:
    case EMC_TASK_STATE_OFF:
    case EMC_TASK_STATE_ON:
        break;
    default:
        set_error(handle, "invalid task state %d", state);
        return -1;
    }
    message.state = static_cast<EMC_TASK_STATE_ENUM>(state);
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_mdi(LcnNmlHandle *handle, const char *block) {
    EMC_TASK_PLAN_EXECUTE message;
    if (!copy_command_text(message.command, sizeof(message.command), block, handle, "MDI block")) {
        return -1;
    }
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_program_close(LcnNmlHandle *handle) {
    EMC_TASK_PLAN_CLOSE message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_program_open(LcnNmlHandle *handle, const char *path) {
    EMC_TASK_PLAN_CLOSE close_message;
    if (send_command(handle, close_message) != 0) {
        return -1;
    }

    EMC_TASK_PLAN_OPEN open_message;
    if (!copy_command_text(open_message.file, sizeof(open_message.file), path, handle,
                           "program path")) {
        return -1;
    }
    return send_command(handle, open_message);
}

extern "C" int32_t lcn_nml_auto_run(LcnNmlHandle *handle, int32_t line) {
    EMC_TASK_PLAN_RUN message;
    message.line = line;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_reset_interpreter(LcnNmlHandle *handle) {
    EMC_TASK_PLAN_INIT message;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_override_limits(LcnNmlHandle *handle) {
    EMC_JOINT_OVERRIDE_LIMITS message;
    message.joint = 0;
    return send_command(handle, message);
}

extern "C" int32_t lcn_nml_load_tool_table(LcnNmlHandle *handle, const char *file) {
    EMC_TOOL_LOAD_TOOL_TABLE message;
    const char *path = file == nullptr ? "" : file;
    if (!copy_command_text(message.file, sizeof(message.file), path, handle, "tool table path")) {
        return -1;
    }
    return send_command(handle, message);
}
