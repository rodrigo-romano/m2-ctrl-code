# Rust crate template for GMT Simulink C controller 

The template directory is the blueprint for importing GMT Simulink C controller into Rust.

Simply copy and rename the `template` directory, then:

 * in `Cargo.toml`, set the package name,
 * copy all the Simulink source (.c) and header (.h) files into the `sys` folder,
 * in `src/lib.rs`, pass the name of the Rust structure that will implement the Rust version of the Simulink controller as argument to `import` (e.g. `AsmPositionner` for the ASM positioner control system),
 * add the path to the new directory into the members list of the workspace section in the root [`Cargo.toml`](../Cargo.toml),


and finally run `cargo build` to check that the new crate is build without errors.

The generated code can be visualized by running [`cargo expand`](https://github.com/dtolnay/cargo-expand).

`simulink_binder::import` macro will generate the following code (taken from the `gmt_m2-ctrl_asm_positionner` crate):
```rust
pub type AsmPositionner = M2P_act_Cfb;
/// Simulink controller wrapper
#[derive(Debug, Default ,Clone ,Copy)]
pub struct M2P_act_Cfb {
    pub inputs: ExtU_M2P_act_Cfb_T,
    pub outputs: ExtY_M2P_act_Cfb_T,
    states: DW_M2P_act_Cfb_T,
}
impl Default for ExtU_M2P_act_Cfb_T {
    fn default() -> Self {
        Self { M2pAct_E: 0f64 }
    }
}
impl Default for ExtY_M2P_act_Cfb_T {
    fn default() -> Self {
        Self { M2pAct_U: 0f64 }
    }
}
impl Default for DW_M2P_act_Cfb_T {
    fn default() -> Self {
        Self {
            M2P_I_rolloffF_states: [0f64; 3usize],
        }
    }
}
impl M2P_act_Cfb {
    /// Creates a new controller
    pub fn new() -> Self {
        let mut this: Self = Default::default();
        let mut data: RT_MODEL_M2P_act_Cfb_T = tag_RTM_M2P_act_Cfb_T {
            dwork: &mut this.states as *mut _,
        };
        unsafe {
            M2P_act_Cfb_initialize(
                &mut data as *mut _,
                &mut this.inputs as *mut _,
                &mut this.outputs as *mut _,
            )
        }
        this
    }
    /// Steps the controller
    pub fn step(&mut self) {
        let mut data: RT_MODEL_M2P_act_Cfb_T = tag_RTM_M2P_act_Cfb_T {
            dwork: &mut self.states as *mut _,
        };
        unsafe {
            M2P_act_Cfb_step(
                &mut data as *mut _,
                &mut self.inputs as *mut _,
                &mut self.outputs as *mut _,
            )
        }
    }
}```