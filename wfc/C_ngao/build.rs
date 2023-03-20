fn main() {
    let sys = simulink_rs::Sys::new(Some("C_temp_ngao"));
    sys.compile().generate_module();
}
