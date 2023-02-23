use std::path::PathBuf;

fn main() {
    cc::Build::new()
        .file("sys/ASM_PIplusD_Fd.c")
        .file("sys/ASM_PIplusD_Fd_data.c")
        .compile("asm_pid_fluid_damping");
    println!(
        "cargo:rustc-link-search=native={}",
        "libasm_pid_fluid_damping"
    );
    println!("cargo:rustc-link-lib=asm_pid_fluid_damping");
    println!("cargo:rerun-if-changed=sys/ASM_PIplusD_Fd.h");

    let bindings = bindgen::builder()
        .header("sys/ASM_PIplusD_Fd.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
