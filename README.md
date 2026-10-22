# linuxcnc-nml

Rust bindings for the LinuxCNC NML command, status, and error channels.

`linuxcnc-nml` is a small Rust wrapper around the native LinuxCNC C++ NML client.
It requires the LinuxCNC development headers and libraries at build time.

Coverage matches the LinuxCNC client surface from `emcmodule.cc`:

- all `linuxcnc.command` methods
- all `linuxcnc.stat` fields, including axes, joints, spindles, synch I/O,
  analog I/O, state tag, offsets, and the tool table
- the `emcError` queue

It does not bind the generic NML/CMS API, NML servers, XDR encoding, or
diagnostics internals.

This is not a pure-Rust implementation of the NML wire protocol. It uses
`libnml` and `liblinuxcnc` in the same way that the official `linuxcnc` Python
module does.

## Crates

- `linuxcnc-nml-sys` — raw C ABI and build glue.
- `linuxcnc-nml` — safe Rust API (`NmlClient`, `Status`, `Command`).

## Build requirements

- Linux
- C++17 compiler
- LinuxCNC headers: default `/usr/include/linuxcnc`
- LinuxCNC libraries: default `/usr/lib`
  - `liblinuxcnc.a`
  - `libnml.so`
  - `liblinuxcncini.so`
  - `libtooldata.so`

Override locations with:

```sh
export LINUXCNC_INCLUDE_DIR=/usr/include/linuxcnc
export LINUXCNC_LIB_DIR=/usr/lib
```

The build script reads `linuxcnc_var LINUXCNCVERSION` when available and
exports it as `linuxcnc_nml::LINUXCNC_VERSION`.

## Example

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

## License

GPL-2.0-or-later. LinuxCNC is GPL-2.0-or-later; linking the native libraries
means distributed binaries must comply with the GPL.

## docs.rs

docs.rs does not provide the LinuxCNC development files, so the native crates
cannot build there by default. Build documentation locally with LinuxCNC
installed:

```sh
cargo doc --no-deps
```

This crate is not affiliated with the LinuxCNC project.
