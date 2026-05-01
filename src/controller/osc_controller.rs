#![no_std]

use core::f32::consts::PI;
use libm::sinf;
use crate::core::dynamic_system::DynamicSystem;

pub struct OscParam {
	pub init_freq: f32,
}

pub struct OscInput {
	pub freq: f32,
	pub gate: bool,     // 是否发声
}

pub struct OscOutput {
	pub sample: f32,
}

pub struct OscState {
	phase: f32,   // 0.0 ~ 1.0
	freq: f32,
	amp: f32,
}

pub struct Oscillator {
	state: OscState,
	input: OscInput,
}

impl DynamicSystem for Oscillator {
	type Param = OscParam;
	type State = OscState;
	type Input = OscInput;
	type Output = OscOutput;

	fn new(param: Self::Param) -> Self {
		Self {
			state: OscState {
				phase: 0.0,
				freq: param.init_freq,
				amp: 0.0,
			},
			input: OscInput {
				freq: param.init_freq,
				gate: false,
			},
		}
	}

	fn set_input(&mut self, input: Self::Input) {
		self.input = input;
	}

	fn update(&mut self, dt: f32) {
		// 更新频率（可随时变）
		self.state.freq = self.input.freq;

		// 相位积分（核心）
		self.state.phase += self.state.freq * dt;

		if self.state.phase >= 1.0 {
			self.state.phase -= 1.0;
		}

		// 简单 gate 包络（避免咔哒声）
		let target_amp = if self.input.gate { 1.0 } else { 0.0 };

		// 平滑逼近（时间常数约 5ms）
		let tau = 0.005;
		let alpha = dt / (tau + dt);

		self.state.amp += (target_amp - self.state.amp) * alpha;
	}

	fn output(&self) -> Self::Output {
		let sample =
			sinf(self.state.phase * 2.0 * PI) * self.state.amp;

		OscOutput { sample }
	}
}
