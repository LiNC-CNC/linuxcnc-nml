# linuxcnc-nml

Safe Rust client for the LinuxCNC NML command, status, and error channels.

This covers the LinuxCNC client surface from `emcmodule.cc`: all
`linuxcnc.command` methods and all `linuxcnc.stat` fields, including the tool
table.

```rust,no_run
use linuxcnc_nml::{Command, NmlClient, TaskMode};

let mut client = NmlClient::open_default()?;
let status = client.poll_status()?;
println!("task mode: {:?}", status.task.mode);

client.send(&Command::SetMode(TaskMode::Mdi))?;
client.wait_complete(std::time::Duration::from_secs(2))?;
client.send(&Command::Mdi("G0 X10".into()))?;
# Ok::<(), linuxcnc_nml::NmlError>(())
```

This is a wrapper around the native NML client, not a pure-Rust wire protocol
implementation. It requires the LinuxCNC development files at build time and a
running LinuxCNC controller at runtime.

See the workspace README for build variables and licensing.
