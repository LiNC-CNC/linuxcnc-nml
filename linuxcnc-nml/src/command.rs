use std::path::PathBuf;

/// LinuxCNC task mode values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskMode {
    Manual,
    Auto,
    Mdi,
    Unknown(i32),
}

impl TaskMode {
    pub fn from_i32(value: i32) -> Self {
        match value {
            1 => Self::Manual,
            2 => Self::Auto,
            3 => Self::Mdi,
            other => Self::Unknown(other),
        }
    }

    pub fn as_i32(self) -> i32 {
        match self {
            Self::Manual => 1,
            Self::Auto => 2,
            Self::Mdi => 3,
            Self::Unknown(value) => value,
        }
    }
}

/// LinuxCNC task state values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskState {
    Estop,
    EstopReset,
    Off,
    On,
    Unknown(i32),
}

impl TaskState {
    pub fn from_i32(value: i32) -> Self {
        match value {
            1 => Self::Estop,
            2 => Self::EstopReset,
            3 => Self::Off,
            4 => Self::On,
            other => Self::Unknown(other),
        }
    }

    pub fn as_i32(self) -> i32 {
        match self {
            Self::Estop => 1,
            Self::EstopReset => 2,
            Self::Off => 3,
            Self::On => 4,
            Self::Unknown(value) => value,
        }
    }
}

/// LinuxCNC trajectory mode values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrajMode {
    Free,
    Coord,
    Teleop,
    Unknown(i32),
}

impl TrajMode {
    pub fn from_i32(value: i32) -> Self {
        match value {
            1 => Self::Free,
            2 => Self::Coord,
            3 => Self::Teleop,
            other => Self::Unknown(other),
        }
    }

    pub fn as_i32(self) -> i32 {
        match self {
            Self::Free => 1,
            Self::Coord => 2,
            Self::Teleop => 3,
            Self::Unknown(value) => value,
        }
    }
}

/// Spindle rotation direction for [`Command::SpindleOn`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpindleDirection {
    Forward,
    Reverse,
}

/// A typed command sent to LinuxCNC's `emcCommand` channel.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    SetDebug(i32),
    TeleopEnable(bool),
    SetTrajMode(TrajMode),
    SetMode(TaskMode),
    SetState(TaskState),

    Mdi(String),
    ProgramOpen(PathBuf),
    AutoRun {
        line: i32,
    },
    AutoPause,
    AutoResume,
    AutoStep,
    AutoReverse,
    AutoForward,
    ResetInterpreter,
    Abort,
    TaskPlanSynch,
    /// Unload the program the controller is holding (`EMC_TASK_PLAN_CLOSE`).
    /// `ProgramOpen` already sends this before opening; as a command of its own
    /// it is what lets an HMI say "nothing is loaded" truthfully.
    ProgramClose,

    Feedrate(f64),
    Rapidrate(f64),
    MaxVelocity(f64),
    SpindleOverride {
        scale: f64,
        spindle: i32,
    },
    SpindleOn {
        direction: SpindleDirection,
        speed: f64,
        spindle: i32,
        wait_for_speed: bool,
    },
    SpindleOff {
        spindle: i32,
    },
    SpindleIncrease {
        spindle: i32,
    },
    SpindleDecrease {
        spindle: i32,
    },
    SpindleConstant {
        spindle: i32,
    },
    SpindleBrake {
        spindle: i32,
        engage: bool,
    },

    ToolOffset {
        toolno: i32,
        z: f64,
        x: f64,
        diameter: f64,
        frontangle: f64,
        backangle: f64,
        orientation: i32,
    },
    Mist(bool),
    Flood(bool),
    OverrideLimits,
    Home(i32),
    Unhome(i32),
    JogStop {
        joint_mode: bool,
        index: i32,
    },
    JogContinuous {
        joint_mode: bool,
        index: i32,
        velocity: f64,
    },
    JogIncrement {
        joint_mode: bool,
        index: i32,
        velocity: f64,
        increment: f64,
    },

    SetOptionalStop(bool),
    SetBlockDelete(bool),
    SetMinLimit {
        joint: i32,
        limit: f64,
    },
    SetMaxLimit {
        joint: i32,
        limit: f64,
    },
    SetFeedOverride(bool),
    SetSpindleOverride {
        on: bool,
        spindle: i32,
    },
    SetFeedHold(bool),
    SetAdaptiveFeed(bool),
    SetDigitalOutput {
        index: u8,
        on: bool,
    },
    SetAnalogOutput {
        index: u8,
        value: f64,
    },

    ErrorMsg(String),
    TextMsg(String),
    DisplayMsg(String),
    LoadToolTable(Option<PathBuf>),
}
