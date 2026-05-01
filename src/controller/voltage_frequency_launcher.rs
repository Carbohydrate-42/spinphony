use crate::core::dynamic_system::DynamicSystem;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum VoltageFrequencyLauncherStateFlag {
    NotStarted,
    Starting,
    Finished,
}

pub struct VoltageFrequencyLauncherParam {
    /// rad/s
    pub velocity_start: f32,
    /// rag/s
    pub velocity_end: f32,
    // s
    pub ramp_time: f32,
    /// v/(rad/s) , under FOC Coordinate system this is Uq/velocity
    pub v_f_ratio: f32,
}


impl Default for VoltageFrequencyLauncherParam {
	fn default() -> Self {
		Self {
			velocity_start: 0.0,
			velocity_end: 0.0,
			ramp_time: 0.0,
			v_f_ratio: 0.0,
		}
	}
}

pub struct VoltageFrequencyLauncherOutput {
    pub uq: f32,
    pub velocity: f32,
}

pub struct VoltageFrequencyLauncherState {
    velocity: f32,
    acceleration: f32,
    elapsed: f32,
    flag: VoltageFrequencyLauncherStateFlag,
}

pub struct VoltageFrequencyLauncher {
    param: VoltageFrequencyLauncherParam,
    state: VoltageFrequencyLauncherState,
}

impl VoltageFrequencyLauncher {
    pub fn finished(&self) -> bool {
        self.state.flag == VoltageFrequencyLauncherStateFlag::Finished
    }

    pub fn launch(&mut self) {
        self.state.flag = VoltageFrequencyLauncherStateFlag::Starting;
        self.state.elapsed = 0.0;
        self.state.velocity = self.param.velocity_start;
    }
}

impl DynamicSystem for VoltageFrequencyLauncher {
    type Param = VoltageFrequencyLauncherParam;
    type State = VoltageFrequencyLauncherState;
    type Input = ();
    type Output = VoltageFrequencyLauncherOutput;

    fn new(param: Self::Param) -> Self {
		// todo : find a better way to use vf_launcher, now here is temporary
        debug_assert!(param.ramp_time >= 0.0);
        let acceleration = (param.velocity_end - param.velocity_start) / param.ramp_time;
        let velocity = param.velocity_start;

        Self {
            param,
            state: VoltageFrequencyLauncherState {
                velocity,
                acceleration,
                elapsed: 0.0,
                flag: VoltageFrequencyLauncherStateFlag::NotStarted,
            },
        }
    }

    fn set_input(&mut self, input: Self::Input) {
        todo!()
    }

    fn update(&mut self, dt: f32) {
        if self.state.flag != VoltageFrequencyLauncherStateFlag::Starting {
            return;
        }

        self.state.elapsed += dt;

        if self.state.elapsed >= self.param.ramp_time {
            self.state.elapsed = self.param.ramp_time;
            self.state.flag = VoltageFrequencyLauncherStateFlag::Finished;
        }

        self.state.velocity =
            self.param.velocity_start + self.state.acceleration * self.state.elapsed;
    }

    fn output(&self) -> Self::Output {
        VoltageFrequencyLauncherOutput {
            uq: self.state.velocity * self.param.v_f_ratio,
            velocity: self.state.velocity,
        }
    }
}
