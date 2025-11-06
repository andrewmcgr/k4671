use anchor_codegen::ConfigBuilder;

fn main() {
    ConfigBuilder::new()
        .entry("src/main.rs")
        .set_version("0.1")
        .set_build_versions("")
        .build();
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
