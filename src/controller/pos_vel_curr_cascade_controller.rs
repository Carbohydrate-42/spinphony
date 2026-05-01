use crate::algorithm::field_oriented_control::FieldOrientedControl;
use crate::controller::motor_controller::{MotorControlMode, MotorController};
use crate::controller::pid_controller::{PIDController, PIDControllerParam};
use crate::core::dynamic_system::DynamicSystem;
use crate::core::frequency_divider::FrequencyDivider;
use core::f32::consts::PI;
use defmt::debug;

pub struct PosVelCurrCascadeControllerParam {
	pub current_loop_pid_param: PIDControllerParam,
	pub velocity_loop_pid_param: PIDControllerParam,
	pub position_loop_pid_param: PIDControllerParam,

	pub mode: MotorControlMode,
}

pub struct PosVelCurrCascadeControllerState {
	uq: f32,
}

pub struct PosVelCurrCascadeControllerInput {
	pub position_ref: f32,
	pub position_meas: f32,

	pub velocity_ref: f32,
	pub velocity_meas: f32,

	pub current_ref: f32,
	pub current_meas: f32,

	pub torque_ref: f32,
}

impl Default for PosVelCurrCascadeControllerInput {
	fn default() -> Self {
		Self {
			position_ref: 0.0,
			position_meas: 0.0,
			velocity_ref: 0.0,
			velocity_meas: 0.0,
			current_ref: 0.0,
			current_meas: 0.0,
			torque_ref: 0.0,
		}
	}
}

impl PosVelCurrCascadeControllerInput {
	#[deprecated]
	pub fn for_velocity_measure_by_position(
		position_meas: f32,
		velocity_ref: f32,
		current_meas: f32,
	) -> Self {
		Self {
			position_meas,
			velocity_ref,
			current_meas,
			..Default::default()
		}
	}

	pub fn for_position(
		position_ref: f32,
		position_meas: f32,
		velocity_meas: f32,
		current_meas: f32,
	) -> Self {
		Self {
			position_ref,
			position_meas,
			velocity_meas,
			current_meas,
			..Default::default()
		}
	}

	pub fn for_velocity(
		velocity_ref: f32,
		position_meas: f32,
		velocity_meas: f32,
		current_meas: f32,
	) -> Self {
		Self {
			position_meas,
			velocity_ref,
			velocity_meas,
			current_meas,
			..Default::default()
		}
	}

	pub fn for_velocity_sensorless(
		velocity_ref: f32,
		position_meas: f32,
		velocity_meas: f32,
		current_meas: f32,
	) -> Self {
		Self {
			position_meas,
			velocity_ref,
			velocity_meas,
			current_meas,
			..Default::default()
		}
	}

	pub fn for_current(
		current_ref: f32,
		current_meas: f32,
	) -> Self {
		Self {
			current_ref,
			current_meas,
			..Default::default()
		}
	}

	pub fn for_velocity_open_loop(
		torque_ref: f32,
		velocity_ref: f32,
	) -> Self {
		Self {
			torque_ref,
			velocity_ref,
			..Default::default()
		}
	}
}

pub struct PosVelCurrCascadeController {
	pub mode: MotorControlMode,

	input: PosVelCurrCascadeControllerInput,
	state: PosVelCurrCascadeControllerState,

	current_loop_pid: PIDController,
	velocity_loop_pid: PIDController,
	position_loop_pid: PIDController,

	current_loop_div: FrequencyDivider,
	velocity_loop_div: FrequencyDivider,
	position_loop_div: FrequencyDivider,
	log_print_div: FrequencyDivider,

}

impl DynamicSystem for PosVelCurrCascadeController {
	type Param = PosVelCurrCascadeControllerParam;
	type State = PosVelCurrCascadeControllerState;
	type Input = PosVelCurrCascadeControllerInput;
	type Output = f32;

	fn new(param: Self::Param) -> Self {
		Self {
			input: PosVelCurrCascadeControllerInput::default(),
			state: PosVelCurrCascadeControllerState { uq: 0.0 },

			current_loop_pid: PIDController::new(param.current_loop_pid_param),
			velocity_loop_pid: PIDController::new(param.velocity_loop_pid_param),
			position_loop_pid: PIDController::new(param.position_loop_pid_param),

			current_loop_div: FrequencyDivider::new(1),
			velocity_loop_div: FrequencyDivider::new(20),
			position_loop_div: FrequencyDivider::new(100),

			log_print_div: FrequencyDivider::new(500),

			mode: param.mode,
		}
	}

	fn set_input(&mut self, input: Self::Input) {
		self.input = input;
	}

	fn update(&mut self, dt: f32) {
		// position-loop
		let velocity_ref = if self.mode == MotorControlMode::Position {
			if self.position_loop_div.tick()
			{
				let pos_dt = dt * self.position_loop_div.div as f32;
				// todo : considering one more position error rectifying mode 'Multi-turn' (多圈定位)
				// 	here is 'Position' Mode (最短路径)
				self.position_loop_pid.set_input(
					FieldOrientedControl::wrap_to_pi(self.input.position_ref - self.input.position_meas)
				);
				self.position_loop_pid.update(pos_dt);
			}
			self.position_loop_pid.output()
		} else {
			self.input.velocity_ref
		};

		// velocity-loop
		let current_ref = if self.mode == MotorControlMode::Position
			|| self.mode == MotorControlMode::Velocity
			// sensorless
			|| self.mode == MotorControlMode::VelocitySensorless
		{
			if self.velocity_loop_div.tick() {
				let vel_dt = dt * self.velocity_loop_div.div as f32;
				self.velocity_loop_pid.set_input(velocity_ref - self.input.velocity_meas);
				self.velocity_loop_pid.update(vel_dt);
			}
			self.velocity_loop_pid.output()
		} else {
			self.input.current_ref
		};

		// current-loop
		let torque_ref = if self.mode == MotorControlMode::Position
			|| self.mode == MotorControlMode::Velocity
			|| self.mode == MotorControlMode::Current
			// sensorless
			|| self.mode == MotorControlMode::VelocitySensorless
		{
			if self.current_loop_div.tick() {
				self.current_loop_pid.set_input(current_ref - self.input.current_meas);
				self.current_loop_pid.update(dt);
			}
			self.current_loop_pid.output()
		} else {
			self.input.torque_ref
		};

		if self.log_print_div.tick() {
			debug!(
				"dt : {}, pos_ref : {}, pos_meas : {}, vel_ref : {} (RPM : {}), vel_meas : {} (RPM : {}), curr_ref : {}, curr_meas : {}",
				dt,
				self.input.position_ref,
				self.input.position_meas,
				velocity_ref,
				velocity_ref / (2.0 * PI) * 60.0,
				self.input.velocity_meas,
				self.input.velocity_meas / (2.0 * PI) * 60.0,
				current_ref,
				self.input.current_meas,
			);
		}

		self.state.uq = torque_ref;
	}

	fn output(&self) -> Self::Output {
		self.state.uq
	}
}
