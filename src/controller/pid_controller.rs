use crate::core::dynamic_system::DynamicSystem;

pub struct PIDControllerParam {
	pub kp: f32,
	pub ki: f32,
	pub kd: f32,
	/// absolute value
	pub output_limit: Option<f32>,
	/// absolute value
	pub integral_limit: Option<f32>,
}

impl Default for PIDControllerParam {
	fn default() -> Self {
		Self {
			kp: 0.0,
			ki: 0.0,
			kd: 0.0,
			output_limit: None,
			integral_limit: None,
		}
	}
}

pub struct PIDControllerState {
	pub integral: f32,
	pub prev_error: f32,
	pub output: f32,
}

pub struct PIDController {
	param: PIDControllerParam,
	state: PIDControllerState,
	/// error
	input: f32,
}

impl PIDController {
	fn compute_p(&self, error: f32) -> f32 {
		self.param.kp * error
	}

	fn compute_d(&mut self, error: f32, dt: f32) -> f32 {
		let derivative = (error - self.state.prev_error) / dt;
		self.state.prev_error = error;
		self.param.kd * derivative
	}

	fn compute_i(&self) -> f32 {
		self.param.ki * self.state.integral
	}

	fn update_integral(&mut self, error: f32, dt: f32) {
		self.state.integral += error * dt;

		if let Some(limit) = self.param.integral_limit {
			self.state.integral = self.state.integral.clamp(-limit, limit);
		}
	}

	fn clamp_output(&self, output: f32) -> f32 {
		if let Some(limit) = self.param.output_limit {
			output.clamp(-limit, limit)
		} else {
			output
		}
	}

	fn is_saturated(&self, output: f32) -> bool {
		if let Some(limit) = self.param.output_limit {
			output > limit || output < -limit
		} else {
			false
		}
	}
}

impl DynamicSystem for PIDController {
	type Param = PIDControllerParam;
	type State = PIDControllerState;
	type Input = f32;
	type Output = f32;

	fn new(param: Self::Param) -> Self {
		Self {
			param,
			input: 0.0,
			state: PIDControllerState {
				integral: 0.0,
				prev_error: 0.0,
				output: 0.0,
			},
		}
	}

	fn set_input(&mut self, input: Self::Input) {
		self.input = input;
	}

	fn update(&mut self, dt: f32) {
		debug_assert!(dt > 0.0);

		let error = self.input;

		let p = self.compute_p(error);
		let d = self.compute_d(error, dt);
		let i = self.compute_i();

		let unsat_output = p + i + d;
		let output = self.clamp_output(unsat_output);

		let saturated = unsat_output != output;

		// anti-windup
		if !saturated
			|| (output > 0.0 && error < 0.0)
			|| (output < 0.0 && error > 0.0)
		{
			self.update_integral(error, dt);
		}

		// redo compute i
		let i = self.compute_i();

		self.state.output = self.clamp_output(p + i + d);
	}

	fn output(&self) -> Self::Output {
		self.state.output
	}
}
