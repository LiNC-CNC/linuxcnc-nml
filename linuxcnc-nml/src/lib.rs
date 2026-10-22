//! Safe Rust client for the LinuxCNC NML command, status, and error channels.
//!
//! The client is a thin wrapper around the native LinuxCNC NML client, the
//! same transport used by the official `linuxcnc` Python module and AXIS.
//!
//! It is not a pure-Rust NML implementation. The crate requires LinuxCNC
//! headers and libraries at build time and LinuxCNC running at runtime.
//!
//! # Example
//!
//! ```no_run
//! use linuxcnc_nml::{Command, NmlClient, TaskMode};
//! use std::time::Duration;
//!
//! # fn main() -> Result<(), linuxcnc_nml::NmlError> {
//! let mut client = NmlClient::open_default()?;
//! let status = client.poll_status()?;
//! println!("mode: {:?}", status.task.mode);
//!
//! client.send(&Command::SetMode(TaskMode::Mdi))?;
//! client.wait_complete(Duration::from_secs(2))?;
//! client.send(&Command::Mdi("G0 X0".to_string()))?;
//! # Ok(())
//! # }
//! ```

#![deny(unsafe_op_in_unsafe_fn)]

pub mod command;
pub mod error;
pub mod status;

pub use command::{Command, SpindleDirection, TaskMode, TaskState, TrajMode};
pub use error::{MessageKind, NmlError, OperatorMessage};
pub use status::{
    AxisStatus, ExecState, InterpState, IoStatus, JointStatus, MotionStatus, OffsetStatus, Pose,
    PositionSet, SpindleStatus, StateTag, Status, TaskStatus, Tool,
};

use std::ffi::{CStr, CString};
use std::os::unix::ffi::OsStrExt;
use std::path::Path;
use std::ptr::NonNull;
use std::time::Duration;

use linuxcnc_nml_sys as sys;

/// LinuxCNC version detected when the native shim was built.
pub const LINUXCNC_VERSION: &str = sys::LINUXCNC_VERSION;

/// A connection to the LinuxCNC `emcStatus`, `emcCommand`, and `emcError`
/// NML channels.
///
/// The handle is not thread-safe and is intentionally not `Sync`. It is
/// `Send` so a worker thread can own it.
pub struct NmlClient {
    raw: NonNull<sys::LcnNmlHandle>,
}

unsafe impl Send for NmlClient {}

impl std::fmt::Debug for NmlClient {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NmlClient").finish_non_exhaustive()
    }
}

impl NmlClient {
    /// Open the default NML configuration file compiled into LinuxCNC.
    pub fn open_default() -> Result<Self, NmlError> {
        let empty = CString::new("").expect("empty CString");
        Self::open_cstring(&empty)
    }

    /// Open a specific NML configuration file.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, NmlError> {
        let bytes = path.as_ref().as_os_str().as_bytes();
        let path = CString::new(bytes)
            .map_err(|_| NmlError::InvalidInput("NML path contains a NUL byte".into()))?;
        Self::open_cstring(&path)
    }

    fn open_cstring(nmlfile: &CStr) -> Result<Self, NmlError> {
        let mut error = [0 as std::ffi::c_char; 512];
        let raw = unsafe { sys::lcn_nml_open(nmlfile.as_ptr(), error.as_mut_ptr(), error.len()) };
        let raw = NonNull::new(raw).ok_or_else(|| {
            let message = unsafe { CStr::from_ptr(error.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            NmlError::Open(if message.is_empty() {
                "could not open LinuxCNC NML channels".into()
            } else {
                message
            })
        })?;
        Ok(Self { raw })
    }

    /// The LinuxCNC version the native shim was built against.
    pub fn version() -> &'static str {
        LINUXCNC_VERSION
    }

    /// Size of the native `EMC_STAT` structure for this LinuxCNC build.
    pub fn stat_size() -> usize {
        unsafe { sys::lcn_nml_stat_size() as usize }
    }

    /// Size of the flat status structure returned by [`Self::poll_status`].
    pub fn status_size() -> usize {
        unsafe { sys::lcn_nml_status_size() as usize }
    }

    /// Poll the status channel and translate the native snapshot.
    pub fn poll_status(&mut self) -> Result<Status, NmlError> {
        let mut raw = sys::LcnNmlStatus::default();
        let result = unsafe { sys::lcn_nml_poll_status(self.raw.as_ptr(), &mut raw) };
        if result != 0 {
            return Err(self.last_error());
        }
        Ok(Status::from_raw(raw))
    }

    /// Poll the error channel.
    ///
    /// Returns `None` when no message is queued. The NML error channel is a
    /// queue: only one client should consume it.
    pub fn poll_error(&mut self) -> Result<Option<OperatorMessage>, NmlError> {
        let mut kind = 0;
        let mut text = [0 as std::ffi::c_char; sys::TEXT_LEN];
        let result = unsafe {
            sys::lcn_nml_poll_error(self.raw.as_ptr(), &mut kind, text.as_mut_ptr(), text.len())
        };
        if result < 0 {
            return Err(self.last_error());
        }
        if result == 0 {
            return Ok(None);
        }
        let text = unsafe { CStr::from_ptr(text.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        Ok(Some(OperatorMessage {
            kind: MessageKind::from_raw(kind),
            text,
        }))
    }

    /// Read the tool table from the LinuxCNC tool mmap/database.
    pub fn tool_table(&mut self) -> Result<Vec<Tool>, NmlError> {
        let mut count = 0usize;
        let result = unsafe {
            sys::lcn_nml_tool_table(self.raw.as_ptr(), std::ptr::null_mut(), 0, &mut count)
        };
        if result != 0 {
            return Err(self.last_error());
        }
        if count == 0 {
            return Ok(Vec::new());
        }

        let mut raw = vec![sys::LcnNmlTool::default(); count];
        let mut actual = 0usize;
        let result = unsafe {
            sys::lcn_nml_tool_table(self.raw.as_ptr(), raw.as_mut_ptr(), raw.len(), &mut actual)
        };
        if result != 0 {
            return Err(self.last_error());
        }
        raw.truncate(actual.min(raw.len()));
        Ok(raw.into_iter().map(Tool::from).collect())
    }

    /// Send one typed machine command.
    pub fn send(&mut self, command: &Command) -> Result<(), NmlError> {
        let result = match command {
            Command::SetDebug(debug) => unsafe {
                sys::lcn_nml_set_debug(self.raw.as_ptr(), *debug)
            },
            Command::TeleopEnable(enable) => unsafe {
                sys::lcn_nml_teleop_enable(self.raw.as_ptr(), *enable as i32)
            },
            Command::SetTrajMode(mode) => unsafe {
                sys::lcn_nml_set_traj_mode(self.raw.as_ptr(), mode.as_i32())
            },
            Command::SetMode(mode) => unsafe {
                sys::lcn_nml_set_mode(self.raw.as_ptr(), mode.as_i32())
            },
            Command::SetState(state) => unsafe {
                sys::lcn_nml_set_state(self.raw.as_ptr(), state.as_i32())
            },
            Command::Mdi(block) => {
                let block = CString::new(block.as_str())
                    .map_err(|_| NmlError::InvalidInput("MDI block contains a NUL byte".into()))?;
                unsafe { sys::lcn_nml_mdi(self.raw.as_ptr(), block.as_ptr()) }
            }
            Command::ProgramOpen(path) => {
                let path = CString::new(path.as_os_str().as_bytes()).map_err(|_| {
                    NmlError::InvalidInput("program path contains a NUL byte".into())
                })?;
                unsafe { sys::lcn_nml_program_open(self.raw.as_ptr(), path.as_ptr()) }
            }
            Command::AutoRun { line } => unsafe { sys::lcn_nml_auto_run(self.raw.as_ptr(), *line) },
            Command::AutoPause => unsafe { sys::lcn_nml_auto_pause(self.raw.as_ptr()) },
            Command::AutoResume => unsafe { sys::lcn_nml_auto_resume(self.raw.as_ptr()) },
            Command::AutoStep => unsafe { sys::lcn_nml_auto_step(self.raw.as_ptr()) },
            Command::AutoReverse => unsafe { sys::lcn_nml_auto_reverse(self.raw.as_ptr()) },
            Command::AutoForward => unsafe { sys::lcn_nml_auto_forward(self.raw.as_ptr()) },
            Command::ResetInterpreter => unsafe {
                sys::lcn_nml_reset_interpreter(self.raw.as_ptr())
            },
            Command::Abort => unsafe { sys::lcn_nml_abort(self.raw.as_ptr()) },
            Command::TaskPlanSynch => unsafe { sys::lcn_nml_task_plan_synch(self.raw.as_ptr()) },
            Command::Feedrate(scale) => unsafe { sys::lcn_nml_feedrate(self.raw.as_ptr(), *scale) },
            Command::Rapidrate(scale) => unsafe {
                sys::lcn_nml_rapidrate(self.raw.as_ptr(), *scale)
            },
            Command::MaxVelocity(velocity) => unsafe {
                sys::lcn_nml_maxvel(self.raw.as_ptr(), *velocity)
            },
            Command::SpindleOverride { scale, spindle } => unsafe {
                sys::lcn_nml_spindle_override(self.raw.as_ptr(), *scale, *spindle)
            },
            Command::SpindleOn {
                direction,
                speed,
                spindle,
                wait_for_speed,
            } => {
                let speed = if *direction == SpindleDirection::Reverse {
                    -*speed
                } else {
                    *speed
                };
                unsafe {
                    sys::lcn_nml_spindle_on(
                        self.raw.as_ptr(),
                        speed,
                        *spindle,
                        *wait_for_speed as i32,
                    )
                }
            }
            Command::SpindleOff { spindle } => unsafe {
                sys::lcn_nml_spindle_off(self.raw.as_ptr(), *spindle)
            },
            Command::SpindleIncrease { spindle } => unsafe {
                sys::lcn_nml_spindle_increase(self.raw.as_ptr(), *spindle)
            },
            Command::SpindleDecrease { spindle } => unsafe {
                sys::lcn_nml_spindle_decrease(self.raw.as_ptr(), *spindle)
            },
            Command::SpindleConstant { spindle } => unsafe {
                sys::lcn_nml_spindle_constant(self.raw.as_ptr(), *spindle)
            },
            Command::SpindleBrake { spindle, engage } => unsafe {
                sys::lcn_nml_spindle_brake(self.raw.as_ptr(), *spindle, *engage as i32)
            },
            Command::ToolOffset {
                toolno,
                z,
                x,
                diameter,
                frontangle,
                backangle,
                orientation,
            } => unsafe {
                sys::lcn_nml_tool_offset(
                    self.raw.as_ptr(),
                    *toolno,
                    *z,
                    *x,
                    *diameter,
                    *frontangle,
                    *backangle,
                    *orientation,
                )
            },
            Command::Mist(on) => unsafe { sys::lcn_nml_mist(self.raw.as_ptr(), *on as i32) },
            Command::Flood(on) => unsafe { sys::lcn_nml_flood(self.raw.as_ptr(), *on as i32) },
            Command::OverrideLimits => unsafe { sys::lcn_nml_override_limits(self.raw.as_ptr()) },
            Command::Home(joint) => unsafe { sys::lcn_nml_home(self.raw.as_ptr(), *joint) },
            Command::Unhome(joint) => unsafe { sys::lcn_nml_unhome(self.raw.as_ptr(), *joint) },
            Command::JogStop { joint_mode, index } => unsafe {
                sys::lcn_nml_jog_stop(self.raw.as_ptr(), *joint_mode as i32, *index)
            },
            Command::JogContinuous {
                joint_mode,
                index,
                velocity,
            } => unsafe {
                sys::lcn_nml_jog_continuous(
                    self.raw.as_ptr(),
                    *joint_mode as i32,
                    *index,
                    *velocity,
                )
            },
            Command::JogIncrement {
                joint_mode,
                index,
                velocity,
                increment,
            } => unsafe {
                sys::lcn_nml_jog_increment(
                    self.raw.as_ptr(),
                    *joint_mode as i32,
                    *index,
                    *velocity,
                    *increment,
                )
            },
            Command::SetOptionalStop(on) => unsafe {
                sys::lcn_nml_set_optional_stop(self.raw.as_ptr(), *on as i32)
            },
            Command::SetBlockDelete(on) => unsafe {
                sys::lcn_nml_set_block_delete(self.raw.as_ptr(), *on as i32)
            },
            Command::SetMinLimit { joint, limit } => unsafe {
                sys::lcn_nml_set_min_limit(self.raw.as_ptr(), *joint, *limit)
            },
            Command::SetMaxLimit { joint, limit } => unsafe {
                sys::lcn_nml_set_max_limit(self.raw.as_ptr(), *joint, *limit)
            },
            Command::SetFeedOverride(on) => unsafe {
                sys::lcn_nml_set_feed_override(self.raw.as_ptr(), *on as i32)
            },
            Command::SetSpindleOverride { on, spindle } => unsafe {
                sys::lcn_nml_set_spindle_override(self.raw.as_ptr(), *on as i32, *spindle)
            },
            Command::SetFeedHold(on) => unsafe {
                sys::lcn_nml_set_feed_hold(self.raw.as_ptr(), *on as i32)
            },
            Command::SetAdaptiveFeed(on) => unsafe {
                sys::lcn_nml_set_adaptive_feed(self.raw.as_ptr(), *on as i32)
            },
            Command::SetDigitalOutput { index, on } => unsafe {
                sys::lcn_nml_set_digital_output(self.raw.as_ptr(), *index as i32, *on as i32)
            },
            Command::SetAnalogOutput { index, value } => unsafe {
                sys::lcn_nml_set_analog_output(self.raw.as_ptr(), *index as i32, *value)
            },
            Command::ErrorMsg(text) => {
                let text = CString::new(text.as_str()).map_err(|_| {
                    NmlError::InvalidInput("operator error contains a NUL byte".into())
                })?;
                unsafe { sys::lcn_nml_error_msg(self.raw.as_ptr(), text.as_ptr()) }
            }
            Command::TextMsg(text) => {
                let text = CString::new(text.as_str()).map_err(|_| {
                    NmlError::InvalidInput("operator text contains a NUL byte".into())
                })?;
                unsafe { sys::lcn_nml_text_msg(self.raw.as_ptr(), text.as_ptr()) }
            }
            Command::DisplayMsg(text) => {
                let text = CString::new(text.as_str()).map_err(|_| {
                    NmlError::InvalidInput("operator display contains a NUL byte".into())
                })?;
                unsafe { sys::lcn_nml_display_msg(self.raw.as_ptr(), text.as_ptr()) }
            }
            Command::LoadToolTable(path) => {
                let path = match path {
                    Some(path) => CString::new(path.as_os_str().as_bytes()).map_err(|_| {
                        NmlError::InvalidInput("tool table path contains a NUL byte".into())
                    })?,
                    None => CString::new("").expect("empty CString"),
                };
                unsafe { sys::lcn_nml_load_tool_table(self.raw.as_ptr(), path.as_ptr()) }
            }
        };
        if result != 0 {
            return Err(self.last_error());
        }
        Ok(())
    }

    /// Wait for the most recently sent command's echo to complete.
    ///
    /// Returns `true` when the command completed, `false` on timeout.
    pub fn wait_complete(&mut self, timeout: Duration) -> Result<bool, NmlError> {
        let seconds = timeout.as_secs_f64();
        let result = unsafe { sys::lcn_nml_wait_complete(self.raw.as_ptr(), seconds) };
        match result {
            1 => Ok(true),
            0 => Ok(false),
            _ => Err(self.last_error()),
        }
    }

    /// Serial number of the most recently sent command.
    pub fn last_serial(&self) -> i32 {
        unsafe { sys::lcn_nml_last_serial(self.raw.as_ptr()) }
    }

    fn last_error(&self) -> NmlError {
        let message = unsafe { sys::last_error(self.raw.as_ptr()) };
        NmlError::Nml(if message.is_empty() {
            "unknown NML error".into()
        } else {
            message
        })
    }
}

impl Drop for NmlClient {
    fn drop(&mut self) {
        unsafe { sys::lcn_nml_close(self.raw.as_ptr()) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_nml_file_is_reported() {
        let path =
            std::env::temp_dir().join(format!("linuxcnc-nml-missing-{}.nml", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let error = NmlClient::open(&path).expect_err("missing NML file must fail");
        assert!(matches!(error, NmlError::Open(_)));
    }

    #[test]
    fn reported_sizes_match_the_raw_layout() {
        assert_eq!(
            NmlClient::status_size(),
            std::mem::size_of::<sys::LcnNmlStatus>()
        );
        assert!(NmlClient::stat_size() >= 10_000);
    }

    #[test]
    #[ignore = "requires a running LinuxCNC controller"]
    fn live_status_smoke() {
        let mut client = NmlClient::open_default().expect("open default NML channels");
        let status = client.poll_status().expect("poll status");
        assert!(status.valid);
        assert_eq!(status.joints.len(), sys::MAX_JOINTS);
        assert_eq!(status.axes.len(), sys::MAX_AXES);
        assert_eq!(status.spindles.len(), sys::MAX_SPINDLES);
    }
}
