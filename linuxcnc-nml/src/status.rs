use linuxcnc_nml_sys as sys;

use crate::command::{TaskMode, TaskState};

/// Interpreter state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterpState {
    Idle,
    Reading,
    Paused,
    Waiting,
    Unknown(i32),
}

impl InterpState {
    pub fn from_i32(value: i32) -> Self {
        match value {
            1 => Self::Idle,
            2 => Self::Reading,
            3 => Self::Paused,
            4 => Self::Waiting,
            other => Self::Unknown(other),
        }
    }
}

/// Task execution state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecState {
    Error,
    Done,
    WaitingForMotion,
    WaitingForMotionQueue,
    WaitingForIo,
    WaitingForMotionAndIo,
    WaitingForDelay,
    WaitingForSystemCommand,
    WaitingForSpindleOriented,
    Unknown(i32),
}

impl ExecState {
    pub fn from_i32(value: i32) -> Self {
        match value {
            1 => Self::Error,
            2 => Self::Done,
            3 => Self::WaitingForMotion,
            4 => Self::WaitingForMotionQueue,
            5 => Self::WaitingForIo,
            7 => Self::WaitingForMotionAndIo,
            8 => Self::WaitingForDelay,
            9 => Self::WaitingForSystemCommand,
            10 => Self::WaitingForSpindleOriented,
            other => Self::Unknown(other),
        }
    }
}

/// A nine-axis LinuxCNC position.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub u: f64,
    pub v: f64,
    pub w: f64,
}

impl From<sys::LcnNmlPose> for Pose {
    fn from(value: sys::LcnNmlPose) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
            a: value.a,
            b: value.b,
            c: value.c,
            u: value.u,
            v: value.v,
            w: value.w,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct StateTag {
    pub fields_float: [f32; sys::MAX_STATE_FLOAT_FIELDS],
    pub fields: [i32; sys::MAX_STATE_FIELDS],
    pub packed_flags: u64,
    pub flags: u64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Tool {
    pub toolno: i32,
    pub pocketno: i32,
    pub offset: Pose,
    pub diameter: f64,
    pub frontangle: f64,
    pub backangle: f64,
    pub orientation: i32,
}

impl From<sys::LcnNmlTool> for Tool {
    fn from(value: sys::LcnNmlTool) -> Self {
        Self {
            toolno: value.toolno,
            pocketno: value.pocketno,
            offset: value.offset.into(),
            diameter: value.diameter,
            frontangle: value.frontangle,
            backangle: value.backangle,
            orientation: value.orientation,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct TaskStatus {
    pub mode: TaskMode,
    pub state: TaskState,
    pub exec_state: ExecState,
    pub interp_state: InterpState,
    pub call_level: i32,
    pub current_line: i32,
    pub read_line: i32,
    pub motion_line: i32,
    pub paused: bool,
    pub interpreter_errcode: i32,
    pub program_units: i32,
    pub queued_mdi_commands: i32,
    pub delay_left: f64,
    pub file: String,
    pub command: String,
    pub ini_filename: String,
    pub optional_stop: bool,
    pub block_delete: bool,
    pub input_timeout: bool,
    pub heartbeat: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MotionStatus {
    pub linear_units: f64,
    pub angular_units: f64,
    pub cycle_time: f64,
    pub joint_count: usize,
    pub spindle_count: usize,
    pub axis_mask: i32,
    pub mode: i32,
    pub enabled: bool,
    pub inpos: bool,
    pub queue: i32,
    pub active_queue: i32,
    pub queue_full: bool,
    pub motion_id: i32,
    pub paused: bool,
    pub feedrate: f64,
    pub rapidrate: f64,
    pub velocity: f64,
    pub acceleration: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub probe_tripped: bool,
    pub probing: bool,
    pub probe_val: i32,
    pub kinematics_type: i32,
    pub motion_type: i32,
    pub distance_to_go: f64,
    pub current_vel: f64,
    pub feed_override_enabled: bool,
    pub adaptive_feed_enabled: bool,
    pub feed_hold_enabled: bool,
    pub heartbeat: u32,
    pub debug: i32,
    pub state_tag: StateTag,
}

#[derive(Debug, Clone, PartialEq)]
pub struct JointStatus {
    pub joint: i32,
    pub joint_type: i32,
    pub units: f64,
    pub backlash: f64,
    pub min_position_limit: f64,
    pub max_position_limit: f64,
    pub max_ferror: f64,
    pub min_ferror: f64,
    pub ferror_current: f64,
    pub ferror_high_mark: f64,
    pub output: f64,
    pub input: f64,
    pub velocity: f64,
    pub inpos: bool,
    pub homing: bool,
    pub homed: bool,
    pub fault: bool,
    pub enabled: bool,
    pub min_soft_limit: bool,
    pub max_soft_limit: bool,
    pub min_hard_limit: bool,
    pub max_hard_limit: bool,
    pub override_limits: bool,
}

impl From<sys::LcnNmlJoint> for JointStatus {
    fn from(value: sys::LcnNmlJoint) -> Self {
        Self {
            joint: value.joint,
            joint_type: value.joint_type,
            units: value.units,
            backlash: value.backlash,
            min_position_limit: value.min_position_limit,
            max_position_limit: value.max_position_limit,
            max_ferror: value.max_ferror,
            min_ferror: value.min_ferror,
            ferror_current: value.ferror_current,
            ferror_high_mark: value.ferror_high_mark,
            output: value.output,
            input: value.input,
            velocity: value.velocity,
            inpos: value.inpos != 0,
            homing: value.homing != 0,
            homed: value.homed != 0,
            fault: value.fault != 0,
            enabled: value.enabled != 0,
            min_soft_limit: value.min_soft_limit != 0,
            max_soft_limit: value.max_soft_limit != 0,
            min_hard_limit: value.min_hard_limit != 0,
            max_hard_limit: value.max_hard_limit != 0,
            override_limits: value.override_limits != 0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct AxisStatus {
    pub axis: i32,
    pub min_position_limit: f64,
    pub max_position_limit: f64,
    pub velocity: f64,
}

impl From<sys::LcnNmlAxis> for AxisStatus {
    fn from(value: sys::LcnNmlAxis) -> Self {
        Self {
            axis: value.axis,
            min_position_limit: value.min_position_limit,
            max_position_limit: value.max_position_limit,
            velocity: value.velocity,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SpindleStatus {
    pub speed: f64,
    pub spindle_scale: f64,
    pub css_maximum: f64,
    pub css_factor: f64,
    pub state: i32,
    pub direction: i32,
    pub brake: i32,
    pub increasing: i32,
    pub enabled: bool,
    pub orient_state: i32,
    pub orient_fault: i32,
    pub spindle_override_enabled: bool,
    pub homed: bool,
}

impl From<sys::LcnNmlSpindle> for SpindleStatus {
    fn from(value: sys::LcnNmlSpindle) -> Self {
        Self {
            speed: value.speed,
            spindle_scale: value.spindle_scale,
            css_maximum: value.css_maximum,
            css_factor: value.css_factor,
            state: value.state,
            direction: value.direction,
            brake: value.brake,
            increasing: value.increasing,
            enabled: value.enabled != 0,
            orient_state: value.orient_state,
            orient_fault: value.orient_fault,
            spindle_override_enabled: value.spindle_override_enabled != 0,
            homed: value.homed != 0,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PositionSet {
    pub commanded: Pose,
    pub actual: Pose,
    pub dtg: Pose,
    pub probed: Pose,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct OffsetStatus {
    pub g5x_index: i32,
    pub g5x_offset: Pose,
    pub g92_offset: Pose,
    pub tool_offset: Pose,
    pub rotation_xy: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct IoStatus {
    pub tool_in_spindle: i32,
    pub pocket_prepped: i32,
    pub tool_from_pocket: i32,
    pub tool_table_current: Tool,
    pub mist: bool,
    pub flood: bool,
    pub estop: bool,
    pub lube: bool,
    pub lube_level: bool,
    pub fault: bool,
    pub reason: i32,
    pub cycle_time: f64,
    pub heartbeat: u32,
    pub debug: i32,
    pub num_extra_joints: i32,
    pub on_soft_limit: bool,
    pub external_offsets_applied: bool,
    pub jogging_active: bool,
}

/// A flat snapshot of the LinuxCNC status channel.
#[derive(Debug, Clone, PartialEq)]
pub struct Status {
    pub valid: bool,
    pub echo_serial_number: i32,
    pub task: TaskStatus,
    pub motion: MotionStatus,
    pub positions: PositionSet,
    pub offsets: OffsetStatus,
    pub joints: Vec<JointStatus>,
    pub axes: Vec<AxisStatus>,
    pub spindles: Vec<SpindleStatus>,
    pub limits: Vec<i32>,
    pub active_gcodes: Vec<i32>,
    pub active_mcodes: Vec<i32>,
    pub active_settings: Vec<f64>,
    pub io: IoStatus,
    pub din: Vec<i32>,
    pub dout: Vec<i32>,
    pub ain: Vec<f64>,
    pub aout: Vec<f64>,
    pub misc_error: Vec<i32>,
    pub eoffset_pose: Pose,
    pub state_tag: StateTag,
    pub command_type: i32,
    pub rcs_status: i32,
    pub rcs_state: i32,
    pub state: i32,
    pub rcs_line: i32,
    pub rcs_source_line: i32,
    pub source_file: String,
    pub debug: i32,
}

impl Status {
    pub fn from_raw(raw: sys::LcnNmlStatus) -> Self {
        let joint_count = (raw.joints.max(0) as usize).min(sys::MAX_JOINTS);
        let spindle_count = (raw.spindles.max(0) as usize).min(sys::MAX_SPINDLES);
        Self {
            valid: raw.valid != 0,
            echo_serial_number: raw.echo_serial_number,
            task: TaskStatus {
                mode: TaskMode::from_i32(raw.task_mode),
                state: TaskState::from_i32(raw.task_state),
                exec_state: ExecState::from_i32(raw.exec_state),
                interp_state: InterpState::from_i32(raw.interp_state),
                call_level: raw.call_level,
                current_line: raw.current_line,
                read_line: raw.read_line,
                motion_line: raw.motion_line,
                paused: raw.task_paused != 0,
                interpreter_errcode: raw.interpreter_errcode,
                program_units: raw.program_units,
                queued_mdi_commands: raw.queued_mdi_commands,
                delay_left: raw.delay_left,
                file: c_array_string(&raw.file),
                command: c_array_string(&raw.command),
                ini_filename: c_array_string(&raw.ini_filename),
                optional_stop: raw.optional_stop != 0,
                block_delete: raw.block_delete != 0,
                input_timeout: raw.input_timeout != 0,
                heartbeat: raw.task_heartbeat as u32,
            },
            motion: MotionStatus {
                linear_units: raw.linear_units,
                angular_units: raw.angular_units,
                cycle_time: raw.cycle_time,
                joint_count,
                spindle_count,
                axis_mask: raw.axis_mask,
                mode: raw.motion_mode,
                enabled: raw.enabled != 0,
                inpos: raw.inpos != 0,
                queue: raw.queue,
                active_queue: raw.active_queue,
                queue_full: raw.queue_full != 0,
                motion_id: raw.motion_id,
                paused: raw.paused != 0,
                feedrate: raw.feedrate,
                rapidrate: raw.rapidrate,
                velocity: raw.velocity,
                acceleration: raw.acceleration,
                max_velocity: raw.max_velocity,
                max_acceleration: raw.max_acceleration,
                probe_tripped: raw.probe_tripped != 0,
                probing: raw.probing != 0,
                probe_val: raw.probe_val,
                kinematics_type: raw.kinematics_type,
                motion_type: raw.motion_type,
                distance_to_go: raw.distance_to_go,
                current_vel: raw.current_vel,
                feed_override_enabled: raw.feed_override_enabled != 0,
                adaptive_feed_enabled: raw.adaptive_feed_enabled != 0,
                feed_hold_enabled: raw.feed_hold_enabled != 0,
                heartbeat: raw.motion_heartbeat as u32,
                debug: raw.motion_debug,
                state_tag: StateTag {
                    fields_float: raw.tag_fields_float,
                    fields: raw.tag_fields,
                    packed_flags: raw.tag_packed_flags,
                    flags: raw.tag_flags,
                },
            },
            positions: PositionSet {
                commanded: raw.position.into(),
                actual: raw.actual_position.into(),
                dtg: raw.dtg.into(),
                probed: raw.probed_position.into(),
            },
            offsets: OffsetStatus {
                g5x_index: raw.g5x_index,
                g5x_offset: raw.g5x_offset.into(),
                g92_offset: raw.g92_offset.into(),
                tool_offset: raw.tool_offset.into(),
                rotation_xy: raw.rotation_xy,
            },
            joints: raw
                .joints_data
                .iter()
                .copied()
                .map(JointStatus::from)
                .collect(),
            axes: raw
                .axes_data
                .iter()
                .copied()
                .map(AxisStatus::from)
                .collect(),
            spindles: raw
                .spindles_data
                .iter()
                .copied()
                .map(SpindleStatus::from)
                .collect(),
            limits: raw
                .joints_data
                .iter()
                .map(|joint| {
                    let mut value = 0;
                    if joint.min_hard_limit != 0 {
                        value |= 1;
                    }
                    if joint.max_hard_limit != 0 {
                        value |= 2;
                    }
                    if joint.min_soft_limit != 0 {
                        value |= 4;
                    }
                    if joint.max_soft_limit != 0 {
                        value |= 8;
                    }
                    value
                })
                .collect(),
            active_gcodes: raw.active_gcodes.to_vec(),
            active_mcodes: raw.active_mcodes.to_vec(),
            active_settings: raw.active_settings.to_vec(),
            io: IoStatus {
                tool_in_spindle: raw.tool_in_spindle,
                pocket_prepped: raw.pocket_prepped,
                tool_from_pocket: raw.tool_from_pocket,
                tool_table_current: raw.tool_table_current.into(),
                mist: raw.mist != 0,
                flood: raw.flood != 0,
                estop: raw.estop != 0,
                lube: raw.lube != 0,
                lube_level: raw.lube_level != 0,
                fault: raw.io_fault != 0,
                reason: raw.io_reason,
                cycle_time: raw.io_cycle_time,
                heartbeat: raw.io_heartbeat,
                debug: raw.io_debug,
                num_extra_joints: raw.num_extra_joints,
                on_soft_limit: raw.on_soft_limit != 0,
                external_offsets_applied: raw.external_offsets_applied != 0,
                jogging_active: raw.jogging_active != 0,
            },
            din: raw.synch_di.to_vec(),
            dout: raw.synch_do.to_vec(),
            ain: raw.analog_input.to_vec(),
            aout: raw.analog_output.to_vec(),
            misc_error: raw.misc_error.to_vec(),
            eoffset_pose: raw.eoffset_pose.into(),
            state_tag: StateTag {
                fields_float: raw.tag_fields_float,
                fields: raw.tag_fields,
                packed_flags: raw.tag_packed_flags,
                flags: raw.tag_flags,
            },
            command_type: raw.command_type,
            rcs_status: raw.rcs_status,
            rcs_state: raw.rcs_state,
            state: raw.rcs_state,
            rcs_line: raw.rcs_line,
            rcs_source_line: raw.rcs_source_line,
            source_file: c_array_string(&raw.source_file),
            debug: raw.debug,
        }
    }
}

fn c_array_string(array: &[std::ffi::c_char]) -> String {
    let bytes: Vec<u8> = array
        .iter()
        .copied()
        .take_while(|value| *value != 0)
        .map(|value| value as u8)
        .collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_status_is_invalid() {
        let status = Status::from_raw(sys::LcnNmlStatus::default());
        assert!(!status.valid);
        assert_eq!(status.task.mode, TaskMode::Unknown(0));
        assert_eq!(status.joints.len(), sys::MAX_JOINTS);
        assert_eq!(status.axes.len(), sys::MAX_AXES);
        assert_eq!(status.spindles.len(), sys::MAX_SPINDLES);
    }

    #[test]
    fn status_strings_stop_at_nul() {
        let mut raw = sys::LcnNmlStatus::default();
        raw.file[0] = b'x' as std::ffi::c_char;
        raw.file[1] = b'y' as std::ffi::c_char;
        raw.file[2] = 0;
        raw.file[3] = b'z' as std::ffi::c_char;
        assert_eq!(c_array_string(&raw.file), "xy");
    }

    #[test]
    #[allow(clippy::field_reassign_with_default)]
    fn extended_fields_are_mapped() {
        let mut raw = sys::LcnNmlStatus::default();
        raw.valid = 1;
        raw.task_mode = 2;
        raw.task_state = 4;
        raw.exec_state = 2;
        raw.interp_state = 1;
        raw.optional_stop = 1;
        raw.block_delete = 1;
        raw.input_timeout = 1;
        raw.task_heartbeat = 11;
        raw.motion_heartbeat = 12;
        raw.io_heartbeat = 13;
        raw.command_type = 509;
        raw.rcs_status = 1;
        raw.rcs_state = 2;
        raw.rcs_line = 3;
        raw.rcs_source_line = 4;
        raw.source_file[0] = b'a' as std::ffi::c_char;
        raw.axes_data[0].axis = 0;
        raw.axes_data[0].velocity = 1.5;
        raw.joints_data[0].joint = 0;
        raw.joints_data[0].homed = 1;
        raw.synch_di[0] = 7;
        raw.synch_do[0] = 8;
        raw.analog_input[0] = 9.0;
        raw.analog_output[0] = 10.0;
        raw.misc_error[0] = 11;
        raw.tag_fields_float[0] = 12.5;
        raw.tag_fields[0] = 13;
        raw.tag_packed_flags = 14;
        raw.tag_flags = 15;
        raw.io_debug = 16;
        raw.motion_debug = 17;
        raw.debug = 18;
        raw.tool_table_current.toolno = 19;

        let status = Status::from_raw(raw);
        assert!(status.valid);
        assert_eq!(status.task.mode, TaskMode::Auto);
        assert_eq!(status.task.state, TaskState::On);
        assert!(status.task.optional_stop);
        assert!(status.task.block_delete);
        assert!(status.task.input_timeout);
        assert_eq!(status.task.heartbeat, 11);
        assert_eq!(status.motion.heartbeat, 12);
        assert_eq!(status.io.heartbeat, 13);
        assert_eq!(status.command_type, 509);
        assert_eq!(status.rcs_status, 1);
        assert_eq!(status.rcs_state, 2);
        assert_eq!(status.rcs_line, 3);
        assert_eq!(status.rcs_source_line, 4);
        assert_eq!(status.source_file, "a");
        assert_eq!(status.axes[0].velocity, 1.5);
        assert!(status.joints[0].homed);
        assert_eq!(status.din[0], 7);
        assert_eq!(status.dout[0], 8);
        assert_eq!(status.ain[0], 9.0);
        assert_eq!(status.aout[0], 10.0);
        assert_eq!(status.misc_error[0], 11);
        assert_eq!(status.state_tag.fields_float[0], 12.5);
        assert_eq!(status.state_tag.fields[0], 13);
        assert_eq!(status.state_tag.packed_flags, 14);
        assert_eq!(status.state_tag.flags, 15);
        assert_eq!(status.io.debug, 16);
        assert_eq!(status.motion.debug, 17);
        assert_eq!(status.debug, 18);
        assert_eq!(status.io.tool_table_current.toolno, 19);
    }
}
