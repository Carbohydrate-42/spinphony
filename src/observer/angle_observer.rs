use crate::core::dynamic_system::DynamicSystem;
use crate::observer::phase_lock_loop_angle_observer::{PhaseLockLoopAngleObserver, PhaseLockLoopAngleObserverInput, PhaseLockLoopAngleObserverOutput, PhaseLockLoopAngleObserverParam};
use crate::observer::sliding_mode_back_emf_observer::{SlidingModeBackEmfInput, SlidingModeBackEmfObserver, SlidingModeBackEmfParam};

pub struct AngleObserverParam {
	pub sliding_mode_back_emf_param: SlidingModeBackEmfParam,
	pub phase_lock_loop_angle_observer_param: PhaseLockLoopAngleObserverParam,
}

pub struct AngleObserver {
	input: SlidingModeBackEmfInput,

	sliding_mode_back_emf_observer: SlidingModeBackEmfObserver,
	phase_lock_loop_angle_observer: PhaseLockLoopAngleObserver,
}

impl DynamicSystem for AngleObserver {
	type Param = AngleObserverParam;
	type State = ();
	type Input = SlidingModeBackEmfInput;
	type Output = PhaseLockLoopAngleObserverOutput;

	fn new(param: Self::Param) -> Self {
		Self {
			input: SlidingModeBackEmfInput {
				i_alpha: 0.0,
				i_beta: 0.0,
				u_alpha: 0.0,
				u_beta: 0.0,
			},
			sliding_mode_back_emf_observer: SlidingModeBackEmfObserver::new(
				param.sliding_mode_back_emf_param
			),
			phase_lock_loop_angle_observer: PhaseLockLoopAngleObserver::new(
				param.phase_lock_loop_angle_observer_param
			),
		}
	}

	fn set_input(&mut self, input: Self::Input) {
		self.input = input;
	}

	fn update(&mut self, dt: f32) {
		// SMO
		self.sliding_mode_back_emf_observer.set_input(SlidingModeBackEmfInput {
			i_alpha: self.input.i_alpha,
			i_beta: self.input.i_beta,
			u_alpha: self.input.u_alpha,
			u_beta: self.input.u_beta,
		});
		self.sliding_mode_back_emf_observer.update(dt);
		let (
			e_alpha,
			e_beta
		) = self.sliding_mode_back_emf_observer.output();

		// PLL
		self.phase_lock_loop_angle_observer.set_input(PhaseLockLoopAngleObserverInput {
			e_alpha,
			e_beta,
		});
		self.phase_lock_loop_angle_observer.update(dt);
	}

	fn output(&self) -> Self::Output {
		// get PLL output
		self.phase_lock_loop_angle_observer.output()
	}
}
