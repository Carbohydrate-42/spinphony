use crate::core::dynamic_system::DynamicSystem;

pub struct LowPass1PoleParam {
	pub tf: f32,
}

pub struct LowPass1PoleState {
	pub y: f32,
}

/// 一阶(单极点)低通
pub struct LowPass1Pole {
	param: LowPass1PoleParam,
	input: f32,
	state: LowPass1PoleState,
}

impl DynamicSystem for LowPass1Pole {
	type Param = LowPass1PoleParam;
	type State = LowPass1PoleState;
	type Input = f32;
	type Output = f32;

	fn new(param: Self::Param) -> Self {
		Self {
			param,
			input: 0.0,
			state: LowPass1PoleState { y: 0.0f32 },
		}
	}

	fn set_input(&mut self, input: Self::Input) {
		self.input = input;
	}

	/// 一阶低通
	///
	/// 连续形式 $dy/dt=(x−y)/Tf$
	///
	/// 离散近似, 令 $dy/dt = (y[k]-y[k-1])/dt$ 得到 $y[k]=ay[k−1]+(1−a)x[k], a=1−dt/Tf$
	fn update(&mut self, dt: f32) {
		if dt <= 0.0 {
			return;
		}
		let alpha = self.param.tf / (self.param.tf + dt);
		self.state.y = alpha * self.state.y + (1.0 - alpha) * self.input;
	}

	fn output(&self) -> Self::Output {
		self.state.y
	}
}
