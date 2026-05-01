use crate::algorithm::field_oriented_control::FieldOrientedControl;
use crate::controller::pid_controller::{PIDController, PIDControllerParam};
use crate::core::dynamic_system::DynamicSystem;
use crate::filter::low_pass_1_pole::{LowPass1Pole, LowPass1PoleParam};

pub struct PhaseLockLoopAngleObserverParam {
	pub pid_param: PIDControllerParam,
}

pub struct PhaseLockLoopAngleObserverState {
	electrical_angle: f32,
	electrical_velocity: f32,
}

pub struct PhaseLockLoopAngleObserverInput {
	pub e_alpha: f32,
	pub e_beta: f32,
}

pub struct PhaseLockLoopAngleObserverOutput {
	pub electrical_angle: f32,
	pub electrical_velocity: f32,
}

pub struct PhaseLockLoopAngleObserver {
	param: PhaseLockLoopAngleObserverParam,
	input: PhaseLockLoopAngleObserverInput,
	state: PhaseLockLoopAngleObserverState,

	low_pass1pole: LowPass1Pole,
	pid_controller: PIDController,
}

impl DynamicSystem for PhaseLockLoopAngleObserver {
	type Param = PhaseLockLoopAngleObserverParam;
	type State = PhaseLockLoopAngleObserverState;
	type Input = PhaseLockLoopAngleObserverInput;
	type Output = PhaseLockLoopAngleObserverOutput;

	fn new(param: Self::Param) -> Self {
		let kp = param.pid_param.kp;
		let ki = param.pid_param.ki;
		let kd = param.pid_param.kd;
		let output_limit = param.pid_param.output_limit;
		let integral_limit = param.pid_param.integral_limit;

		Self {
			param,
			input: PhaseLockLoopAngleObserverInput {
				e_alpha: 0.0,
				e_beta: 0.0,
			},
			state: PhaseLockLoopAngleObserverState {
				electrical_angle: 0.0,
				electrical_velocity: 0.0,
			},
			low_pass1pole: LowPass1Pole::new(LowPass1PoleParam {
				tf: 0.0008,
			}),
			pid_controller: PIDController::new(PIDControllerParam {
				kp,
				ki,
				kd,
				output_limit,
				integral_limit,
			}),
		}
	}

	fn set_input(&mut self, input: Self::Input) {
		self.input = input;
	}

	fn update(&mut self, dt: f32) {
		let s = &mut self.state;
		let i = &self.input;
		///
		/// Ed = Eα cosθ + Eβ sinθ
		///
		/// |E| = √(Eα² + Eβ²)
		///
		/// Back-EMF magnitude |E| ∝ ω
		///
		/// Ed = |E| * sin(θ_error)
		///
		/// small angle:
		/// sin(θ_error) ≈ θ_error
		///
		/// -> ed ≈ |E| * θ_error
		///
		/// -> θ_error ≈ ed / |E|
		///
		/// ignore |E|
		///
		/// -> θ_error ≈ ed
		///
		let e_d = 0.0;
		// adopting park-transform to Back-EMF
		let (e_d_est, _) = FieldOrientedControl::park(i.e_alpha, i.e_beta, s.electrical_angle);
		// θ_error
		let theta_error = e_d - e_d_est;

		self.pid_controller.set_input(theta_error);
		self.pid_controller.update(dt);
		let electrical_velocity = self.pid_controller.output();

		self.low_pass1pole.set_input(electrical_velocity);
		self.low_pass1pole.update(dt);
		s.electrical_velocity = self.low_pass1pole.output();

		s.electrical_angle += dt * s.electrical_velocity;
		s.electrical_angle = FieldOrientedControl::wrap_to_pi(s.electrical_angle);
	}

	fn output(&self) -> Self::Output {
		PhaseLockLoopAngleObserverOutput {
			electrical_angle: self.state.electrical_angle,
			electrical_velocity: self.state.electrical_velocity,
		}
	}
}
