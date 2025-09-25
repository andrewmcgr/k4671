fn main() {
    anchor_codegen::ConfigBuilder::new()
        .entry("src/main.rs")
        .set_version("k4671 0.1")
        .set_build_versions("")
        .build();
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
