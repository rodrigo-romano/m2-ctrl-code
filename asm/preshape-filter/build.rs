use std::path::PathBuf;

fn main() {
    cc::Build::new()
        .file("sys/ASM_preshapeBesselF.c")
        .file("sys/ASM_preshapeBesselF_data.c")
        .compile("asm_preshape_bessel_filter");
    println!(
        "cargo:rustc-link-search=native={}",
        "libasm_preshape_bessel_filter"
    );
    println!("cargo:rustc-link-lib=asm_preshape_bessel_filter");
    println!("cargo:rerun-if-changed=sys/ASM_preshapeBesselF.h");

    let bindings = bindgen::builder()
        .header("sys/ASM_preshapeBesselF.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
