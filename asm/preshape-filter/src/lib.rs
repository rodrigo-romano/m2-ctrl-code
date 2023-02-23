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
                &mut this.outputs as *mut _,
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
                &mut self.outputs as *mut _,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use matio_rs::MatFile;

    #[test]
    fn step() {
        let mat = MatFile::load("../../simulink_models/m2asm_tests.mat").unwrap();
        let preshape_bessel_step_t: Vec<f64> = mat.var("preshapeBessel_step_t").unwrap();
        let preshape_bessel_step_y: Vec<f64> = mat.var("preshapeBessel_step_y").unwrap();

        let n = preshape_bessel_step_t.len();

        let mut ctrl = AsmPreshapeFilter::new();
        ctrl.inputs.AO_cmd = 1f64;
        let mut y = vec![];
        for _ in &preshape_bessel_step_t {
            ctrl.step();
            y.push(ctrl.outputs.cmd_f);
            y.push(ctrl.outputs.cmd_f_dot);
            y.push(ctrl.outputs.cmd_f_ddot);
        }

        let y_err = (preshape_bessel_step_y
            .chunks(n)
            .take(1)
            .zip(preshape_bessel_step_y.chunks(n).skip(1).take(1))
            .zip(preshape_bessel_step_y.chunks(n).skip(2).take(1))
            .flat_map(|((x, dx), ddx)| {
                x.iter()
                    .zip(dx)
                    .zip(ddx)
                    .flat_map(|((x, dx), ddx)| vec![*x, *dx, *ddx])
            })
            .zip(&y)
            .map(|(sim_y, y)| sim_y - y)
            .map(|x| x * x)
            .sum::<f64>()
            / ((3 * n) as f64))
            .sqrt();
        assert!(dbg!(y_err) < 1e-8);
    }
}
