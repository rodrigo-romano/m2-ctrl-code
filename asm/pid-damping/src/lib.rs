#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

/// Simulink controller wrapper
#[derive(Debug, Clone, Copy, Default)]
pub struct AsmPidDamping {
    /// Inputs Simulink structure
    pub inputs: ExtU_ASM_PIplusD_Fd_T,
    /// Outputs Simulink structure
    pub outputs: ExtY_ASM_PIplusD_Fd_T,
    states: DW_ASM_PIplusD_Fd_T,
}
impl Default for ExtU_ASM_PIplusD_Fd_T {
    fn default() -> Self {
        ExtU_ASM_PIplusD_Fd_T {
            asm_SP: 0f64,
            asm_FB: 0f64,
        }
    }
}
impl Default for ExtY_ASM_PIplusD_Fd_T {
    fn default() -> Self {
        ExtY_ASM_PIplusD_Fd_T {
            asm_Ufb: 0f64,
            asm_Fd: 0f64,
        }
    }
}
impl Default for DW_ASM_PIplusD_Fd_T {
    fn default() -> Self {
        DW_ASM_PIplusD_Fd_T {
            ASMPIcontroller_states: 0f64,
            Numericaldifferentiation_states: 0f64,
        }
    }
}
impl AsmPidDamping {
    /// Creates a new controller
    pub fn new() -> Self {
        let mut this: Self = Default::default();
        let mut data: RT_MODEL_ASM_PIplusD_Fd_T = tag_RTM_ASM_PIplusD_Fd_T {
            dwork: &mut this.states as *mut _,
        };
        unsafe {
            ASM_PIplusD_Fd_initialize(
                &mut data as *mut _,
                &mut this.inputs as *mut _,
                &mut this.outputs as &mut _,
            )
        }
        this
    }
    /// Steps the controller
    pub fn step(&mut self) {
        let mut data: RT_MODEL_ASM_PIplusD_Fd_T = tag_RTM_ASM_PIplusD_Fd_T {
            dwork: &mut self.states as *mut _,
        };
        unsafe {
            ASM_PIplusD_Fd_step(
                &mut data as *mut _,
                &mut self.inputs as *mut _,
                &mut self.outputs as &mut _,
            )
        }
    }
}
