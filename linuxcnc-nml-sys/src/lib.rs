//! Raw bindings for the LinuxCNC NML command, status, and error channels.
//!
//! This crate builds a small C++ shim against the installed LinuxCNC headers
//! and links the native `liblinuxcnc` / `libnml` libraries. It is Linux-only
//! and requires the LinuxCNC development files at build time.
//!
//! The public structs are a flat snapshot of the parts of `EMC_STAT` that a
//! user interface normally needs. They are deliberately independent of the
//! C++ class layout so callers do not need a C++ compiler to use the crate.

#![allow(non_camel_case_types)]

use std::ffi::{c_char, c_int};

pub const MAX_JOINTS: usize = 16;
pub const MAX_AXES: usize = 9;
pub const MAX_SPINDLES: usize = 8;
pub const MAX_DIO: usize = 64;
pub const MAX_AIO: usize = 64;
pub const MAX_MISC_ERROR: usize = 64;
pub const MAX_GCODES: usize = 17;
pub const MAX_MCODES: usize = 10;
pub const MAX_SETTINGS: usize = 5;
pub const MAX_STATE_FIELDS: usize = 8;
pub const MAX_STATE_FLOAT_FIELDS: usize = 5;
pub const MAX_TOOLS: usize = 1001;
pub const TEXT_LEN: usize = 256;
pub const SOURCE_FILE_LEN: usize = 64;

/// LinuxCNC version detected at build time.
pub const LINUXCNC_VERSION: &str = env!("LINUXCNC_NML_VERSION");

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LcnNmlPose {
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

impl Default for LcnNmlPose {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LcnNmlTool {
    pub toolno: i32,
    pub pocketno: i32,
    pub offset: LcnNmlPose,
    pub diameter: f64,
    pub frontangle: f64,
    pub backangle: f64,
    pub orientation: i32,
}

impl Default for LcnNmlTool {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LcnNmlJoint {
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
    pub inpos: i32,
    pub homing: i32,
    pub homed: i32,
    pub fault: i32,
    pub enabled: i32,
    pub min_soft_limit: i32,
    pub max_soft_limit: i32,
    pub min_hard_limit: i32,
    pub max_hard_limit: i32,
    pub override_limits: i32,
}

impl Default for LcnNmlJoint {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LcnNmlAxis {
    pub axis: i32,
    pub min_position_limit: f64,
    pub max_position_limit: f64,
    pub velocity: f64,
}

impl Default for LcnNmlAxis {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LcnNmlSpindle {
    pub speed: f64,
    pub spindle_scale: f64,
    pub css_maximum: f64,
    pub css_factor: f64,
    pub state: i32,
    pub direction: i32,
    pub brake: i32,
    pub increasing: i32,
    pub enabled: i32,
    pub orient_state: i32,
    pub orient_fault: i32,
    pub spindle_override_enabled: i32,
    pub homed: i32,
}

impl Default for LcnNmlSpindle {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct LcnNmlStatus {
    pub valid: i32,
    pub echo_serial_number: i32,

    pub task_mode: i32,
    pub task_state: i32,
    pub exec_state: i32,
    pub interp_state: i32,
    pub call_level: i32,
    pub current_line: i32,
    pub read_line: i32,
    pub motion_line: i32,
    pub task_paused: i32,
    pub interpreter_errcode: i32,
    pub program_units: i32,
    pub queued_mdi_commands: i32,
    pub delay_left: f64,
    pub file: [c_char; TEXT_LEN],
    pub command: [c_char; TEXT_LEN],
    pub ini_filename: [c_char; TEXT_LEN],

    pub linear_units: f64,
    pub angular_units: f64,
    pub cycle_time: f64,
    pub joints: i32,
    pub spindles: i32,
    pub axis_mask: i32,
    pub motion_mode: i32,
    pub enabled: i32,
    pub inpos: i32,
    pub queue: i32,
    pub active_queue: i32,
    pub queue_full: i32,
    pub motion_id: i32,
    pub paused: i32,
    pub feedrate: f64,
    pub rapidrate: f64,
    pub velocity: f64,
    pub acceleration: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub probe_tripped: i32,
    pub probing: i32,
    pub probe_val: i32,
    pub kinematics_type: i32,
    pub motion_type: i32,
    pub distance_to_go: f64,
    pub current_vel: f64,
    pub feed_override_enabled: i32,
    pub adaptive_feed_enabled: i32,
    pub feed_hold_enabled: i32,

    pub position: LcnNmlPose,
    pub actual_position: LcnNmlPose,
    pub dtg: LcnNmlPose,
    pub probed_position: LcnNmlPose,

    pub g5x_offset: LcnNmlPose,
    pub g92_offset: LcnNmlPose,
    pub tool_offset: LcnNmlPose,
    pub g5x_index: i32,
    pub rotation_xy: f64,

    pub active_gcodes: [i32; MAX_GCODES],
    pub active_mcodes: [i32; MAX_MCODES],
    pub active_settings: [f64; MAX_SETTINGS],

    pub joints_data: [LcnNmlJoint; MAX_JOINTS],
    pub axes_data: [LcnNmlAxis; MAX_AXES],
    pub spindles_data: [LcnNmlSpindle; MAX_SPINDLES],

    pub tool_in_spindle: i32,
    pub pocket_prepped: i32,
    pub tool_from_pocket: i32,
    pub tool_table_current: LcnNmlTool,
    pub mist: i32,
    pub flood: i32,
    pub estop: i32,
    pub lube: i32,
    pub lube_level: i32,
    pub io_fault: i32,
    pub io_reason: i32,
    pub io_cycle_time: f64,

    pub num_extra_joints: i32,
    pub on_soft_limit: i32,
    pub external_offsets_applied: i32,
    pub jogging_active: i32,

    pub command_type: i32,
    pub rcs_status: i32,
    pub rcs_state: i32,
    pub rcs_line: i32,
    pub rcs_source_line: i32,
    pub source_file: [c_char; SOURCE_FILE_LEN],

    pub optional_stop: i32,
    pub block_delete: i32,
    pub input_timeout: i32,
    pub debug: i32,
    pub task_heartbeat: i32,
    pub motion_heartbeat: i32,
    pub io_heartbeat: u32,
    pub motion_debug: i32,
    pub io_debug: i32,

    pub synch_di: [i32; MAX_DIO],
    pub synch_do: [i32; MAX_DIO],
    pub analog_input: [f64; MAX_AIO],
    pub analog_output: [f64; MAX_AIO],
    pub misc_error: [i32; MAX_MISC_ERROR],
    pub eoffset_pose: LcnNmlPose,

    pub tag_fields_float: [f32; MAX_STATE_FLOAT_FIELDS],
    pub tag_fields: [i32; MAX_STATE_FIELDS],
    pub tag_packed_flags: u64,
    pub tag_flags: u64,
}

impl Default for LcnNmlStatus {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
pub struct LcnNmlHandle {
    _private: [u8; 0],
}

extern "C" {
    pub fn lcn_nml_open(
        nmlfile: *const c_char,
        error: *mut c_char,
        error_len: usize,
    ) -> *mut LcnNmlHandle;
    pub fn lcn_nml_close(handle: *mut LcnNmlHandle);

    pub fn lcn_nml_last_error(handle: *const LcnNmlHandle) -> *const c_char;
    pub fn lcn_nml_status_size() -> u32;
    pub fn lcn_nml_stat_size() -> u32;
    pub fn lcn_nml_version() -> *const c_char;

    pub fn lcn_nml_poll_status(handle: *mut LcnNmlHandle, out: *mut LcnNmlStatus) -> i32;
    pub fn lcn_nml_poll_error(
        handle: *mut LcnNmlHandle,
        kind: *mut i32,
        text: *mut c_char,
        text_len: usize,
    ) -> i32;
    pub fn lcn_nml_wait_complete(handle: *mut LcnNmlHandle, timeout_seconds: f64) -> i32;
    pub fn lcn_nml_last_serial(handle: *const LcnNmlHandle) -> i32;
    pub fn lcn_nml_tool_table(
        handle: *mut LcnNmlHandle,
        out: *mut LcnNmlTool,
        capacity: usize,
        count: *mut usize,
    ) -> i32;

    pub fn lcn_nml_set_debug(handle: *mut LcnNmlHandle, debug: i32) -> i32;
    pub fn lcn_nml_teleop_enable(handle: *mut LcnNmlHandle, enable: i32) -> i32;
    pub fn lcn_nml_set_traj_mode(handle: *mut LcnNmlHandle, mode: i32) -> i32;
    pub fn lcn_nml_set_mode(handle: *mut LcnNmlHandle, mode: i32) -> i32;
    pub fn lcn_nml_set_state(handle: *mut LcnNmlHandle, state: i32) -> i32;
    pub fn lcn_nml_mdi(handle: *mut LcnNmlHandle, block: *const c_char) -> i32;
    pub fn lcn_nml_program_open(handle: *mut LcnNmlHandle, path: *const c_char) -> i32;
    pub fn lcn_nml_auto_run(handle: *mut LcnNmlHandle, line: i32) -> i32;
    pub fn lcn_nml_auto_pause(handle: *mut LcnNmlHandle) -> i32;
    pub fn lcn_nml_auto_resume(handle: *mut LcnNmlHandle) -> i32;
    pub fn lcn_nml_auto_step(handle: *mut LcnNmlHandle) -> i32;
    pub fn lcn_nml_auto_reverse(handle: *mut LcnNmlHandle) -> i32;
    pub fn lcn_nml_auto_forward(handle: *mut LcnNmlHandle) -> i32;
    pub fn lcn_nml_reset_interpreter(handle: *mut LcnNmlHandle) -> i32;
    pub fn lcn_nml_abort(handle: *mut LcnNmlHandle) -> i32;
    pub fn lcn_nml_task_plan_synch(handle: *mut LcnNmlHandle) -> i32;

    pub fn lcn_nml_feedrate(handle: *mut LcnNmlHandle, scale: f64) -> i32;
    pub fn lcn_nml_rapidrate(handle: *mut LcnNmlHandle, scale: f64) -> i32;
    pub fn lcn_nml_maxvel(handle: *mut LcnNmlHandle, velocity: f64) -> i32;
    pub fn lcn_nml_spindle_override(handle: *mut LcnNmlHandle, scale: f64, spindle: i32) -> i32;
    pub fn lcn_nml_spindle_on(
        handle: *mut LcnNmlHandle,
        speed: f64,
        spindle: i32,
        wait_for_speed: i32,
    ) -> i32;
    pub fn lcn_nml_spindle_off(handle: *mut LcnNmlHandle, spindle: i32) -> i32;
    pub fn lcn_nml_spindle_increase(handle: *mut LcnNmlHandle, spindle: i32) -> i32;
    pub fn lcn_nml_spindle_decrease(handle: *mut LcnNmlHandle, spindle: i32) -> i32;
    pub fn lcn_nml_spindle_constant(handle: *mut LcnNmlHandle, spindle: i32) -> i32;
    pub fn lcn_nml_spindle_brake(handle: *mut LcnNmlHandle, spindle: i32, engage: i32) -> i32;

    pub fn lcn_nml_tool_offset(
        handle: *mut LcnNmlHandle,
        toolno: i32,
        z: f64,
        x: f64,
        diameter: f64,
        frontangle: f64,
        backangle: f64,
        orientation: i32,
    ) -> i32;
    pub fn lcn_nml_mist(handle: *mut LcnNmlHandle, on: i32) -> i32;
    pub fn lcn_nml_flood(handle: *mut LcnNmlHandle, on: i32) -> i32;
    pub fn lcn_nml_override_limits(handle: *mut LcnNmlHandle) -> i32;
    pub fn lcn_nml_home(handle: *mut LcnNmlHandle, joint: i32) -> i32;
    pub fn lcn_nml_unhome(handle: *mut LcnNmlHandle, joint: i32) -> i32;
    pub fn lcn_nml_jog_stop(handle: *mut LcnNmlHandle, joint_mode: i32, index: i32) -> i32;
    pub fn lcn_nml_jog_continuous(
        handle: *mut LcnNmlHandle,
        joint_mode: i32,
        index: i32,
        velocity: f64,
    ) -> i32;
    pub fn lcn_nml_jog_increment(
        handle: *mut LcnNmlHandle,
        joint_mode: i32,
        index: i32,
        velocity: f64,
        increment: f64,
    ) -> i32;

    pub fn lcn_nml_set_optional_stop(handle: *mut LcnNmlHandle, on: i32) -> i32;
    pub fn lcn_nml_set_block_delete(handle: *mut LcnNmlHandle, on: i32) -> i32;
    pub fn lcn_nml_set_min_limit(handle: *mut LcnNmlHandle, joint: i32, limit: f64) -> i32;
    pub fn lcn_nml_set_max_limit(handle: *mut LcnNmlHandle, joint: i32, limit: f64) -> i32;
    pub fn lcn_nml_set_feed_override(handle: *mut LcnNmlHandle, on: i32) -> i32;
    pub fn lcn_nml_set_spindle_override(handle: *mut LcnNmlHandle, on: i32, spindle: i32) -> i32;
    pub fn lcn_nml_set_feed_hold(handle: *mut LcnNmlHandle, on: i32) -> i32;
    pub fn lcn_nml_set_adaptive_feed(handle: *mut LcnNmlHandle, on: i32) -> i32;
    pub fn lcn_nml_set_digital_output(handle: *mut LcnNmlHandle, index: i32, on: i32) -> i32;
    pub fn lcn_nml_set_analog_output(handle: *mut LcnNmlHandle, index: i32, value: f64) -> i32;

    pub fn lcn_nml_error_msg(handle: *mut LcnNmlHandle, text: *const c_char) -> i32;
    pub fn lcn_nml_text_msg(handle: *mut LcnNmlHandle, text: *const c_char) -> i32;
    pub fn lcn_nml_display_msg(handle: *mut LcnNmlHandle, text: *const c_char) -> i32;
    pub fn lcn_nml_load_tool_table(handle: *mut LcnNmlHandle, file: *const c_char) -> i32;
}

/// Read the last error from a native NML handle.
///
/// # Safety
///
/// `handle` must be a live handle returned by [`lcn_nml_open`] and not yet
/// passed to [`lcn_nml_close`].
pub unsafe fn last_error(handle: *const LcnNmlHandle) -> String {
    unsafe {
        let ptr = lcn_nml_last_error(handle);
        if ptr.is_null() {
            return String::new();
        }
        std::ffi::CStr::from_ptr(ptr).to_string_lossy().into_owned()
    }
}

pub fn version() -> String {
    unsafe {
        let ptr = lcn_nml_version();
        if ptr.is_null() {
            return String::new();
        }
        std::ffi::CStr::from_ptr(ptr).to_string_lossy().into_owned()
    }
}

#[allow(dead_code)]
fn _assert_c_int() {
    let _: c_int = 0;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_status_layout_matches_native() {
        let native = unsafe { lcn_nml_status_size() };
        assert_eq!(std::mem::size_of::<LcnNmlStatus>() as u32, native);
        assert!(unsafe { lcn_nml_stat_size() } >= 10_000);
    }
}
