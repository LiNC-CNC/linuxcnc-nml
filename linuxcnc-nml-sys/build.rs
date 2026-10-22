use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=shim/lcn_nml.cc");
    println!("cargo:rerun-if-changed=shim/lcn_nml.h");
    println!("cargo:rerun-if-env-changed=LINUXCNC_INCLUDE_DIR");
    println!("cargo:rerun-if-env-changed=LINUXCNC_LIB_DIR");
    println!("cargo:rerun-if-env-changed=LINUXCNC_VERSION");

    let include_dir = env::var_os("LINUXCNC_INCLUDE_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/usr/include/linuxcnc"));
    let lib_dir = env::var_os("LINUXCNC_LIB_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/usr/lib"));

    require(
        &include_dir.join("emc_nml.hh"),
        "LinuxCNC headers",
        "LINUXCNC_INCLUDE_DIR",
    );
    require(
        &lib_dir.join("liblinuxcnc.a"),
        "LinuxCNC static library",
        "LINUXCNC_LIB_DIR",
    );

    let version = env::var("LINUXCNC_VERSION")
        .ok()
        .filter(|value| !value.trim().is_empty())
        .or_else(detect_version)
        .unwrap_or_else(|| "unknown".to_string());

    let version_define = format!("\"{}\"", version.replace('\\', "\\\\").replace('"', "\\\""));
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .file("shim/lcn_nml.cc")
        .include(&include_dir)
        .flag_if_supported("-std=c++17")
        .flag_if_supported("-Wno-ignored-qualifiers")
        .warnings(true)
        .define("LCN_NML_VERSION", Some(version_define.as_str()));
    build.compile("lcn_nml_shim");

    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=static=linuxcnc");
    println!("cargo:rustc-link-lib=dylib=nml");
    println!("cargo:rustc-link-lib=dylib=linuxcncini");
    println!("cargo:rustc-link-lib=dylib=tooldata");
    println!("cargo:rustc-link-lib=dylib=stdc++");
    println!("cargo:rustc-link-lib=dylib=m");
    println!("cargo:rustc-link-lib=dylib=pthread");
    println!("cargo:rustc-env=LINUXCNC_NML_VERSION={version}");
}

fn require(path: &Path, what: &str, variable: &str) {
    if !path.exists() {
        panic!(
            "{what} not found at {}; set {variable} to the LinuxCNC location",
            path.display()
        );
    }
}

fn detect_version() -> Option<String> {
    let output = Command::new("linuxcnc_var")
        .arg("LINUXCNCVERSION")
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let version = String::from_utf8(output.stdout).ok()?;
    let version = version.trim();
    if version.is_empty() {
        None
    } else {
        Some(version.to_string())
    }
}
