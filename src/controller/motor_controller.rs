use crate::algorithm::field_oriented_control::FieldOrientedControl;
use crate::controller::pos_vel_curr_cascade_controller::{PosVelCurrCascadeController, PosVelCurrCascadeControllerInput, PosVelCurrCascadeControllerParam};
use crate::controller::voltage_frequency_launcher::{VoltageFrequencyLauncher, VoltageFrequencyLauncherParam};
use crate::core::dynamic_system::DynamicSystem;
use crate::core::execution_limiter::ExecutionLimiter;
use defmt::info;
use libm::fabsf;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorControlMode {
	Position,
	Velocity,
	Current,

	VelocityOpenLoop,

	/// todo : move v/f launching process into MotorController
	/// 	and make VelocityOpenLoop to VelocitySensorless transition process smoothly
	VelocitySensorless,

	// not implemented
	PositionSensorless,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorTransitionState {
	OpenLoop,
	Transition,
	ClosedLoop,
}

pub struct MotorControllerParam {
	pub position_velocity_current_cascade_controller_param: PosVelCurrCascadeControllerParam,

	pub voltage_frequency_launcher_param: VoltageFrequencyLauncherParam,

	pub voltage_power_supply: f32,
	pub pole_pairs: u8,

	/// mechanical angle /rad
	pub zero_position_angle: f32,
}

pub struct MotorControllerState {
	/// (ua, ub, uc)
	triple_phase_voltage_with_vbus_bias: (f32, f32, f32),
	pub triple_phase_voltage: (f32, f32, f32),

	/// (ud, uq, theta)
	pub park_result: (f32, f32, f32),

	/// theta for open-loop
	theta_open_loop: f32,

	// state for VelocitySensorless
	motor_transition_state: MotorTransitionState,
	blend: f32,
	uq_open_loop: f32,
	uq_closed_loop: f32,
	theta_offset: f32,
}

pub struct MotorControllerInput {
	pub position_velocity_current_cascade_controller_input: PosVelCurrCascadeControllerInput,
}

pub struct MotorController {
	position_velocity_current_cascade_controller: PosVelCurrCascadeController,

	input: MotorControllerInput,
	pub state: MotorControllerState,

	voltage_power_supply: f32,
	pole_pairs: u8,
	zero_position_angle: f32,

	pub foc: FieldOrientedControl,

	voltage_frequency_launcher: VoltageFrequencyLauncher,
	execution_limiter: ExecutionLimiter,
}

impl MotorController {
	pub fn set_mode(&mut self, mode: MotorControlMode) {
		self.position_velocity_current_cascade_controller.mode = mode;
	}

	pub fn set_zero_position_angle(&mut self, zero_position_angle: f32) {
		self.zero_position_angle = zero_position_angle;
	}

	pub fn get_zero_position_angle(&self) -> f32 {
		self.zero_position_angle
	}

	pub fn get_theta_by_mech_angle(&mut self, position_meas: f32) -> f32 {
		let mech_rel = FieldOrientedControl::wrap_to_pi(position_meas - self.zero_position_angle);
		FieldOrientedControl::wrap_to_pi(mech_rel * (self.pole_pairs as f32))
	}
}


impl MotorController {
	pub fn closed_loop(&mut self, dt: f32) -> (f32, f32, f32) {
		let position_meas = self.input.position_velocity_current_cascade_controller_input.position_meas;
		// Uq
		let uq = {
			self.position_velocity_current_cascade_controller.set_input(
				PosVelCurrCascadeControllerInput {
					..self.input.position_velocity_current_cascade_controller_input
				}
			);
			self.position_velocity_current_cascade_controller.update(dt);
			self.position_velocity_current_cascade_controller.output()
		};

		// theta
		let theta = self.get_theta_by_mech_angle(position_meas);

		(0.0, uq, theta)
	}

	pub fn open_loop(&mut self, dt: f32) -> (f32, f32, f32) {
		// Uq
		let uq = {
			self.input.position_velocity_current_cascade_controller_input.torque_ref
		};

		// theta
		let theta = {
			let velocity_ref = self.input.position_velocity_current_cascade_controller_input.velocity_ref;
			self.state.theta_open_loop += (self.pole_pairs as f32) * velocity_ref * dt;
			self.state.theta_open_loop = FieldOrientedControl::wrap_to_pi(self.state.theta_open_loop);
			self.state.theta_open_loop
		};

		(0.0, uq, theta)
	}


	/// todo : optimize this code
	pub fn closed_loop_sensorless(&mut self, dt: f32) -> (f32, f32, f32) {
		let input = &self.input.position_velocity_current_cascade_controller_input;

		let position_meas = input.position_meas;
		let velocity_ref = input.velocity_ref;
		let velocity_meas = input.velocity_meas;

		let theta_smo = FieldOrientedControl::wrap_to_pi(
			position_meas * (self.pole_pairs as f32)
		);

		self.voltage_frequency_launcher.update(dt);
		let vf = self.voltage_frequency_launcher.output();

		let open_loop_velocity = if self.voltage_frequency_launcher.finished() {
			velocity_ref   // 切换后用目标速度
		} else {
			vf.velocity
		};

		self.state.theta_open_loop += (self.pole_pairs as f32) * open_loop_velocity * dt;
		self.state.theta_open_loop =
			FieldOrientedControl::wrap_to_pi(self.state.theta_open_loop);

		self.position_velocity_current_cascade_controller.set_input(
			PosVelCurrCascadeControllerInput::for_velocity(
				velocity_ref,
				position_meas,
				velocity_meas,
				input.current_meas,
			)
		);

		self.position_velocity_current_cascade_controller.update(dt);

		self.state.uq_closed_loop =
			self.position_velocity_current_cascade_controller.output();

		let (uq, theta) = match self.state.motor_transition_state {
			MotorTransitionState::OpenLoop => {
				self.state.uq_open_loop = vf.uq;


				let theta_err =
					FieldOrientedControl::wrap_to_pi(self.state.theta_open_loop - theta_smo);

				let uq_err =
					self.state.uq_open_loop - self.state.uq_closed_loop;


				if (
					self.voltage_frequency_launcher.finished()
						// todo : consider better threshold param for theta_err and uq_err
						&& fabsf(theta_err) <= 0.05 // ~3°
						&& fabsf(uq_err) <= 0.005
				) {
					self.state.theta_offset =
						self.state.theta_open_loop - theta_smo;

					self.state.blend = 0.0;
					self.state.motor_transition_state =
						MotorTransitionState::Transition;
				}

				(vf.uq, self.state.theta_open_loop)
			}

			MotorTransitionState::Transition => {

				// ===== blend =====
				self.state.blend += dt / 0.2;
				if self.state.blend > 1.0 {
					self.state.blend = 1.0;
				}

				// ===== θ 对齐 =====
				let theta_smo_aligned =
					FieldOrientedControl::wrap_to_pi(
						theta_smo + self.state.theta_offset
					);

				let theta = FieldOrientedControl::wrap_to_pi(
					(1.0 - self.state.blend) * self.state.theta_open_loop +
						self.state.blend * theta_smo_aligned
				);

				// ===== uq 融合（关键）=====
				let uq = (1.0 - self.state.blend) * self.state.uq_open_loop
					+ self.state.blend * self.state.uq_closed_loop;

				if self.state.blend >= 1.0 {
					self.state.motor_transition_state =
						MotorTransitionState::ClosedLoop;
				}

				(uq, theta)
			}
			MotorTransitionState::ClosedLoop => {
				if self.execution_limiter.tick() {
					info!("transition to closed loop");
				}
				let theta =
					FieldOrientedControl::wrap_to_pi(
						theta_smo + self.state.theta_offset
					);

				let uq = self.state.uq_closed_loop;

				(uq, theta)
			}
		};

		(0.0, uq, theta)
	}

	/// closed_loop_sensorless_raw
	///
	/// closed_loop_sensorless_raw is not ugly, there is a rigid pause when transit from 'OpenLoop' to 'ClosedLoop'
	#[deprecated]
	pub fn closed_loop_sensorless_raw(&mut self, dt: f32) -> (f32, f32, f32) {
		let position_meas = self.input.position_velocity_current_cascade_controller_input.position_meas;

		self.voltage_frequency_launcher.update(dt);
		let vf_launcher_output = self.voltage_frequency_launcher.output();

		let (ud, uq, theta) = if (self.voltage_frequency_launcher.finished()) {
			// Uq
			let uq = {
				self.position_velocity_current_cascade_controller.set_input(
					PosVelCurrCascadeControllerInput {
						..self.input.position_velocity_current_cascade_controller_input
					}
				);
				self.position_velocity_current_cascade_controller.update(dt);
				self.position_velocity_current_cascade_controller.output()
			};

			// theta
			let theta = {
				FieldOrientedControl::wrap_to_pi(position_meas * (self.pole_pairs as f32))
			};

			(0.0, uq, theta)
		} else {
			self.set_input(
				MotorControllerInput {
					position_velocity_current_cascade_controller_input:
					PosVelCurrCascadeControllerInput::for_velocity_open_loop(
						vf_launcher_output.uq,
						vf_launcher_output.velocity,
					)
				}
			);
			self.open_loop(dt)
		};


		(ud, uq, theta)
	}
}


impl DynamicSystem for MotorController {
	type Param = MotorControllerParam;
	type State = MotorControllerState;
	type Input = MotorControllerInput;
	type Output = (f32, f32, f32);

	fn new(param: Self::Param) -> Self {
		let mut _self = Self {
			position_velocity_current_cascade_controller: PosVelCurrCascadeController::new(
				param.position_velocity_current_cascade_controller_param
			),
			input: MotorControllerInput {
				position_velocity_current_cascade_controller_input: Default::default(),
			},

			state: MotorControllerState {
				triple_phase_voltage_with_vbus_bias: (0.0, 0.0, 0.0),
				triple_phase_voltage: (0.0, 0.0, 0.0),
				park_result: (0.0, 0.0, 0.0),
				theta_open_loop: 0.0,
				motor_transition_state: MotorTransitionState::OpenLoop,
				blend: 0.0,
				uq_open_loop: 0.0,
				uq_closed_loop: 0.0,
				theta_offset: 0.0,
			},
			voltage_power_supply: param.voltage_power_supply,
			pole_pairs: param.pole_pairs,
			zero_position_angle: param.zero_position_angle,

			foc: FieldOrientedControl::new(param.voltage_power_supply),

			voltage_frequency_launcher: VoltageFrequencyLauncher::new(param.voltage_frequency_launcher_param),

			execution_limiter: ExecutionLimiter::new(1),
		};

		_self.voltage_frequency_launcher.launch();
		_self
	}

	fn set_input(&mut self, input: Self::Input) {
		self.input = input;
	}

	fn update(&mut self, dt: f32) {
		let (ud, uq, theta) = match self.position_velocity_current_cascade_controller.mode {
			// closed loop
			MotorControlMode::Position => self.closed_loop(dt),
			MotorControlMode::Velocity => self.closed_loop(dt),
			MotorControlMode::Current => self.closed_loop(dt),
			// open loop
			MotorControlMode::VelocityOpenLoop => self.open_loop(dt),
			// closed loop sensorless
			MotorControlMode::VelocitySensorless => self.closed_loop_sensorless(dt),
			MotorControlMode::PositionSensorless => self.closed_loop_sensorless(dt)
		};

		// pwm tuning
		self.state.park_result = (ud, uq, theta);
		self.state.triple_phase_voltage = self.foc.svpwm(
			ud,
			uq,
			theta,
		);
		self.state.triple_phase_voltage_with_vbus_bias = self.foc.svpwm_with_vbus_bias(
			ud,
			uq,
			theta,
		);
	}

	fn output(&self) -> Self::Output {
		self.state.triple_phase_voltage_with_vbus_bias
	}
}
