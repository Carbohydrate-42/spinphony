use crate::algorithm::field_oriented_control::FieldOrientedControl;
use crate::core::dynamic_system::DynamicSystem;
use crate::filter::low_pass_1_pole::{LowPass1Pole, LowPass1PoleParam};

pub struct AngularVelocityCalculatorParam {
	pub low_pass1_pole_param: LowPass1PoleParam
}

pub struct AngularVelocityCalculatorState {
	/// rad
	angle_prev: f32,
	// rad
	angle_sum: f32,
	// rad/s
	angular_velocity: f32
}

pub struct AngularVelocityCalculator {
	state: AngularVelocityCalculatorState,
	/// angle current / rad
	input: f32,

	low_pass_1_pole: LowPass1Pole
}

impl AngularVelocityCalculator {
	pub fn get_angle_sum(&mut self) -> f32 {
		self.state.angle_sum
	}
}

impl DynamicSystem for AngularVelocityCalculator {
	type Param = AngularVelocityCalculatorParam;
	type State = AngularVelocityCalculatorState;
	type Input = f32;
	type Output = f32;

	fn new(param: Self::Param) -> Self {
		Self {
			state: AngularVelocityCalculatorState {
				angle_prev: 0.0,
				angle_sum: 0.0,
				angular_velocity: 0.0,
			},
			input: 0.0,
			low_pass_1_pole: LowPass1Pole::new(
				LowPass1PoleParam {
					tf: param.low_pass1_pole_param.tf
				}
			),
		}
	}

	fn set_input(&mut self, input: Self::Input) {
		self.input = input;
	}

	fn update(&mut self, dt: f32) {
		let angle_delta = FieldOrientedControl::angle_diff(self.input, self.state.angle_prev);
		self.state.angle_sum += angle_delta;

		self.state.angular_velocity = angle_delta / dt;

		// adopt low-pass-filter
		self.low_pass_1_pole.set_input(self.state.angular_velocity);
		self.low_pass_1_pole.update(dt);
		self.state.angular_velocity = self.low_pass_1_pole.output();

		self.state.angle_prev = self.input;
	}

	fn output(&self) -> Self::Output {
		self.state.angular_velocity
	}
}
