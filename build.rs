use cc;

fn main() {
    cc::Build::new()
        .file("src/positionner/M2_POS_Control.c")
        .file("src/positionner/M2_POS_Control_data.c")
        .compile("m2_pos_controller");
    cc::Build::new()
        .file("src/ptt_asmC_fd/pttASMC_FluidD.c")
        .file("src/ptt_asmC_fd/pttASMC_FluidD_data.c")
        .compile("m2_ptt_asmC_fd");    
    /*    
    cc::Build::new()
        .file("src/piezostack/M2_PZT_Control.c")
        .file("src/piezostack/M2_PZT_Control_data.c")
        .compile("m2_pzt_controller");
    cc::Build::new()
        .file("src/tiptilt/TT_Control.c")
        .file("src/tiptilt/TT_Control_data.c")
        .compile("m2_tt_controller");
     */
}
