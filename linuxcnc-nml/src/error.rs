use thiserror::Error;

/// Errors returned by [`crate::NmlClient`].
#[derive(Debug, Error)]
pub enum NmlError {
    #[error("could not open LinuxCNC NML channels: {0}")]
    Open(String),
    #[error("NML error: {0}")]
    Nml(String),
    #[error("invalid input: {0}")]
    InvalidInput(String),
}

/// The kind of message read from the `emcError` queue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageKind {
    OperatorError,
    OperatorText,
    OperatorDisplay,
    NmlError,
    NmlText,
    NmlDisplay,
    Unknown(i32),
}

impl MessageKind {
    pub fn from_raw(kind: i32) -> Self {
        match kind {
            11 => Self::OperatorError,
            12 => Self::OperatorText,
            13 => Self::OperatorDisplay,
            1 => Self::NmlError,
            2 => Self::NmlText,
            3 => Self::NmlDisplay,
            other => Self::Unknown(other),
        }
    }

    pub fn is_error(self) -> bool {
        matches!(
            self,
            Self::OperatorError | Self::NmlError | Self::Unknown(_)
        )
    }
}

/// One message from the NML error channel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OperatorMessage {
    pub kind: MessageKind,
    pub text: String,
}
