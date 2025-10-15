use anchor_codegen::ConfigBuilder;
use std::process::Command;

fn main() {
    let githash = Command::new("git")
        .args(&["rev-parse", "--short", "HEAD"])
        .output()
        .expect("Failed to execute git command");
    let githash = String::from_utf8(githash.stdout)
        .expect("Failed to convert git output to UTF-8")
        .trim() // Remove trailing newline
        .to_string();
    println!("cargo:rustc-env=GIT_HASH={}", githash);
    println!("cargo:rerun-if-changed=.git/HEAD");

    ConfigBuilder::new()
        .entry("src/main.rs")
        .set_version("0.1")
        .set_build_versions(&githash)
        .build();
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
