use crate::core::dynamic_system::DynamicSystem;
use crate::filter::low_pass_1_pole::{LowPass1Pole, LowPass1PoleParam};

pub struct SlidingModeBackEmfParam {
	/// one shunt Inductance /H
	pub l_s: f32,
	/// one shunt resistance /Ohm
	pub r_s: f32,
	/// smo gain
	pub smo_h: f32,
	pub smo_limit: f32,
}

pub struct SlidingModeBackEmfInput {
	pub i_alpha: f32,
	pub i_beta: f32,
	pub u_alpha: f32,
	pub u_beta: f32,
}

pub struct SlidingModeBackEmfState {
	e_alpha: f32,
	e_beta: f32,

	e_alpha_filtered: f32,
	e_beta_filtered: f32,

	i_alpha_est: f32,
	i_beta_est: f32,
}

pub struct SlidingModeBackEmfObserver {
	param: SlidingModeBackEmfParam,
	input: SlidingModeBackEmfInput,
	state: SlidingModeBackEmfState,

	low_pass1pole_alpha: LowPass1Pole,
	low_pass1pole_beta: LowPass1Pole,
}

impl SlidingModeBackEmfObserver {
	#[inline]
	fn sat(err: f32, limit: f32) -> f32 {
		if err > limit {
			1.0
		} else if err < -limit {
			-1.0
		} else {
			err / limit
		}
	}
}

impl DynamicSystem for SlidingModeBackEmfObserver {
	type Param = SlidingModeBackEmfParam;
	type State = SlidingModeBackEmfState;
	type Input = SlidingModeBackEmfInput;
	type Output = (f32, f32);

	fn new(param: Self::Param) -> Self {
		Self {
			param,
			input: SlidingModeBackEmfInput {
				i_alpha: 0.0,
				i_beta: 0.0,
				u_alpha: 0.0,
				u_beta: 0.0,
			},
			state: SlidingModeBackEmfState {
				e_alpha: 0.0,
				e_beta: 0.0,
				e_alpha_filtered: 0.0,
				e_beta_filtered: 0.0,
				i_alpha_est: 0.0,
				i_beta_est: 0.0,
			},
			low_pass1pole_alpha: LowPass1Pole::new(LowPass1PoleParam { tf: 0.001 }),
			low_pass1pole_beta: LowPass1Pole::new(LowPass1PoleParam { tf: 0.001 }),
		}
	}

	fn set_input(&mut self, input: Self::Input) {
		self.input = input;
	}


	/// todo : reduce seamless code, alpha and beta axis have the same calculation progress
	fn update(&mut self, dt: f32) {
		let p = &self.param;
		let s = &mut self.state;
		let i = &self.input;

		// current estimate model : E_a = V_a - I_a * R_s - L_s * d(I_a)/dt
		s.i_alpha_est += dt * (-p.r_s / p.l_s * s.i_alpha_est + (i.u_alpha - s.e_alpha) / p.l_s);
		s.i_beta_est += dt * (-p.r_s / p.l_s * s.i_beta_est + (i.u_beta - s.e_beta) / p.l_s);

		// current error
		let i_alpha_err = s.i_alpha_est - i.i_alpha;
		let i_beta_err = s.i_beta_est - i.i_beta;

		// SMO
		s.e_alpha = p.smo_h * Self::sat(i_alpha_err, p.smo_limit);
		s.e_beta = p.smo_h * Self::sat(i_beta_err, p.smo_limit);

		// low pass filter [output directly]
		self.low_pass1pole_alpha.set_input(s.e_alpha);
		self.low_pass1pole_alpha.update(dt);
		s.e_alpha_filtered = self.low_pass1pole_alpha.output();

		self.low_pass1pole_beta.set_input(s.e_beta);
		self.low_pass1pole_beta.update(dt);
		s.e_beta_filtered = self.low_pass1pole_beta.output();
	}

	fn output(&self) -> Self::Output {
		(self.state.e_alpha_filtered, self.state.e_beta_filtered)
	}
}
