# linuxcnc-nml-sys

Raw C ABI and build glue for the LinuxCNC NML command, status, and error
channels.

This crate compiles a small C++ shim against the installed LinuxCNC headers and
links `liblinuxcnc.a` plus `libnml`, `liblinuxcncini`, and `libtooldata`.

Use [`linuxcnc-nml`](https://crates.io/crates/linuxcnc-nml) for the safe Rust
API.

## Build requirements

- Linux
- C++17 compiler
- LinuxCNC headers and libraries

Defaults:

- headers: `/usr/include/linuxcnc`
- libraries: `/usr/lib`

Override with `LINUXCNC_INCLUDE_DIR` and `LINUXCNC_LIB_DIR`.

## License

GPL-2.0-or-later.
