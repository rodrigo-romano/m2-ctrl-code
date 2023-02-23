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
            asm_FF: 0f64,
        }
    }
}
impl Default for ExtY_ASM_PIplusD_Fd_T {
    fn default() -> Self {
        ExtY_ASM_PIplusD_Fd_T {
            asm_U: 0f64,
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

#[cfg(test)]
mod tests {
    use super::*;
    use matio_rs::MatFile;

    #[test]
    fn impulse() {
        let mat = MatFile::load("../../simulink_models/m2asm_tests.mat").unwrap();
        let asm_fb_t: Vec<f64> = mat.var("asm_fb_t").unwrap();
        let asm_fb_y: Vec<f64> = mat.var("asm_fb_y").unwrap();

        let n = asm_fb_t.len();

        let sim_y: Vec<_> = asm_fb_y
            .chunks(n)
            .take(1)
            .zip(asm_fb_y.chunks(n).skip(1).take(1))
            .flat_map(|(x, dx)| x.iter().zip(dx).flat_map(|(x, dx)| vec![*x, *dx]))
            .collect();
        dbg!(&sim_y[..10]);

        let mut ctrl = AsmPidDamping::new();
        let mut y = vec![];
        for i in 0..n {
            ctrl.inputs.asm_FB = if i == 0 { 8000f64 } else { 0f64 };
            ctrl.step();
            y.push(ctrl.outputs.asm_U);
            y.push(ctrl.outputs.asm_Fd);
        }
        dbg!(&y[..10]);

        let y_err = (asm_fb_y
            .chunks(n)
            .take(1)
            .zip(asm_fb_y.chunks(n).skip(1).take(1))
            .flat_map(|(x, dx)| x.iter().zip(dx).flat_map(|(x, dx)| vec![*x, *dx]))
            .zip(&y)
            .map(|(sim_y, y)| sim_y - y)
            .map(|x| x * x)
            .sum::<f64>()
            / ((3 * n) as f64))
            .sqrt();
        assert!(dbg!(y_err) < 1e-6);
    }
}
