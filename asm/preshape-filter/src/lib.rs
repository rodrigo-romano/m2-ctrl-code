#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

/// Simulink controller wrapper
#[derive(Debug, Clone, Copy, Default)]
pub struct AsmPreshapeFilter {
    /// Inputs Simulink structure
    pub inputs: ExtU_ASM_preshapeBesselF_T,
    /// Outputs Simulink structure
    pub outputs: ExtY_ASM_preshapeBesselF_T,
    states: DW_ASM_preshapeBesselF_T,
}
impl Default for ExtU_ASM_preshapeBesselF_T {
    fn default() -> Self {
        ExtU_ASM_preshapeBesselF_T { AO_cmd: 0f64 }
    }
}
impl Default for ExtY_ASM_preshapeBesselF_T {
    fn default() -> Self {
        ExtY_ASM_preshapeBesselF_T {
            cmd_f_ddot: 0f64,
            cmd_f_dot: 0f64,
            cmd_f: 0f64,
        }
    }
}
impl Default for DW_ASM_preshapeBesselF_T {
    fn default() -> Self {
        DW_ASM_preshapeBesselF_T {
            SSflag_d_DSTATE: [0f64; 4],
        }
    }
}
impl AsmPreshapeFilter {
    /// Creates a new controller
    pub fn new() -> Self {
        let mut this: Self = Default::default();
        let mut data: RT_MODEL_ASM_preshapeBesselF_T = tag_RTM_ASM_preshapeBesselF_T {
            dwork: &mut this.states as *mut _,
        };
        unsafe {
            ASM_preshapeBesselF_initialize(
                &mut data as *mut _,
                &mut this.inputs as *mut _,
                &mut this.outputs as &mut _,
            )
        }
        this
    }
    /// Steps the controller
    pub fn step(&mut self) {
        let mut data: RT_MODEL_ASM_preshapeBesselF_T = tag_RTM_ASM_preshapeBesselF_T {
            dwork: &mut self.states as *mut _,
        };
        unsafe {
            ASM_preshapeBesselF_step(
                &mut data as *mut _,
                &mut self.inputs as *mut _,
                &mut self.outputs as &mut _,
            )
        }
    }
}
