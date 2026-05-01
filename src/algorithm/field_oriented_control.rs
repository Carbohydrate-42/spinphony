use core::f32::consts::PI;
use libm::{atan2f, cosf, floorf, sinf, sqrtf};

pub struct FieldOrientedControl {
	vbus: f32,
}


impl FieldOrientedControl {
	/// 1.0 / sqrtf(3.0)
	pub const INV_SQRT_3: f32 = 0.577_350_269_189_625_8;

	/// sqrtf(3.0)
	pub const SQRT_3: f32 = 1.732_050_807_568_877_2;

	pub fn new(vbus: f32) -> Self {
		Self {
			vbus
		}
	}

	#[inline]
	pub fn wrap_to_pi(angle: f32) -> f32 {
		let two_pi = 2.0 * PI;
		angle - two_pi * floorf((angle + PI) / two_pi)
	}

	#[inline]
	pub fn wrap_to_0_to_2pi(mut angle: f32) -> f32 {
		const TWO_PI: f32 = core::f32::consts::PI * 2.0;

		while angle > TWO_PI {
			angle -= TWO_PI;
		}
		while angle < 0.0 {
			angle += TWO_PI;
		}
		angle
	}

	#[inline]
	pub fn angle_diff(angle_a: f32, angle_b: f32) -> f32 {
		Self::wrap_to_pi(angle_a - angle_b)
	}

	#[inline]
	pub fn apply_vbus_bias(&self, ua: f32, ub: f32, uc: f32) -> (f32, f32, f32) {
		let vbus = self.vbus;
		let half = vbus * 0.5;

		let ua = (ua + half).clamp(0.0, vbus);
		let ub = (ub + half).clamp(0.0, vbus);
		let uc = (uc + half).clamp(0.0, vbus);

		(ua, ub, uc)
	}

	#[inline]
	pub fn clarke(a: f32, b: f32, _c: f32) -> (f32, f32) {
		// ia + ib + ic = 0
		let alpha = a;
		let beta = (a + 2.0 * b) * Self::INV_SQRT_3;
		(alpha, beta)
	}

	#[inline]
	pub fn inverse_clarke(alpha: f32, beta: f32) -> (f32, f32, f32) {
		let a = alpha;
		let b = (Self::SQRT_3 * beta - alpha) * 0.5;
		let c = -a - b;
		(a, b, c)
	}

	#[inline]
	pub fn park(alpha: f32, beta: f32, theta: f32) -> (f32, f32) {
		let s = sinf(theta);
		let c = cosf(theta);

		let d = alpha * c + beta * s;
		let q = -alpha * s + beta * c;

		(d, q)
	}

	#[inline]
	pub fn inverse_park(d: f32, q: f32, theta: f32) -> (f32, f32) {
		let s = sinf(theta);
		let c = cosf(theta);

		let alpha = d * c - q * s;
		let beta = d * s + q * c;

		(alpha, beta)
	}

	pub fn foc_raw(&self, ud: f32, uq: f32, theta: f32) -> (f32, f32, f32) {
		// dq -> αβ
		let (alpha, beta) = Self::inverse_park(ud, uq, theta);
		// αβ -> abc
		let (ua, ub, uc) = Self::inverse_clarke(alpha, beta);
		self.apply_vbus_bias(ua, ub, uc)
	}

	pub fn foc_spwm(&self, ud: f32, uq: f32, theta: f32) -> (f32, f32, f32) {
		let vbus = self.vbus;
		let max_voltage = vbus * 0.5;

		let magnitude = sqrtf(ud * ud + uq * uq);

		let (ud, uq) = if magnitude > max_voltage {
			let scale = max_voltage / magnitude;
			(ud * scale, uq * scale)
		} else {
			(ud, uq)
		};

		let (ua, ub, uc) = self.foc_raw(ud, uq, theta);
		self.apply_vbus_bias(ua, ub, uc)
	}

	/// svpwm classic (经典扇区法)
	pub fn svpwm_classic(&self, ud: f32, uq: f32, theta: f32) -> (f32, f32, f32) {
		let vbus = self.vbus;

		let (u_alpha, u_beta) = Self::inverse_park(ud, uq, theta);

		let mut magnitude = sqrtf(u_alpha * u_alpha + u_beta * u_beta);
		let max_voltage = vbus * 0.57735026919;

		if magnitude > max_voltage {
			magnitude = max_voltage;
		}

		if magnitude < 1e-6 {
			return self.apply_vbus_bias(0.0, 0.0, 0.0);
		}

		let mut angle = atan2f(u_beta, u_alpha);
		if angle < 0.0 {
			angle += 2.0 * PI;
		}

		let pi_3 = PI / 3.0;

		let sector = ((floorf(angle / pi_3) as i32) % 6) + 1;

		let t1 = Self::SQRT_3
			* sinf(sector as f32 * pi_3 - angle)
			* magnitude
			/ vbus;

		let t2 = Self::SQRT_3
			* sinf(angle - (sector as f32 - 1.0) * pi_3)
			* magnitude
			/ vbus;

		let t0 = 1.0 - t1 - t2;

		let (ta, tb, tc) = match sector {
			1 => (t1 + t2 + t0 * 0.5, t2 + t0 * 0.5, t0 * 0.5),
			2 => (t1 + t0 * 0.5, t1 + t2 + t0 * 0.5, t0 * 0.5),
			3 => (t0 * 0.5, t1 + t2 + t0 * 0.5, t2 + t0 * 0.5),
			4 => (t0 * 0.5, t1 + t0 * 0.5, t1 + t2 + t0 * 0.5),
			5 => (t2 + t0 * 0.5, t0 * 0.5, t1 + t2 + t0 * 0.5),
			6 => (t1 + t2 + t0 * 0.5, t0 * 0.5, t1 + t0 * 0.5),
			_ => (0.0, 0.0, 0.0),
		};

		let ua = ta * vbus;
		let ub = tb * vbus;
		let uc = tc * vbus;

		self.apply_vbus_bias(ua, ub, uc)
	}

	/// svpwm min-max tuning (Min-Max 零序注入)
	///
	/// 与 '经典方法' 等价, 不过更简单, 且工程上常用
	pub fn svpwm_with_vbus_bias(&self, ud: f32, uq: f32, theta: f32) -> (f32, f32, f32) {
		let (ua, ub, uc) = self.svpwm(ud, uq, theta);
		self.apply_vbus_bias(ua, ub, uc)
	}

	pub fn svpwm(&self, ud: f32, uq: f32, theta: f32) -> (f32, f32, f32) {
		let vbus = self.vbus;
		let max_voltage = vbus * Self::INV_SQRT_3;

		let magnitude = sqrtf(ud * ud + uq * uq);

		let (ud, uq) = if magnitude > max_voltage {
			let scale = max_voltage / magnitude;
			(ud * scale, uq * scale)
		} else {
			(ud, uq)
		};

		let (mut ua, mut ub, mut uc) = self.foc_raw(ud, uq, theta);

		let max = ua.max(ub.max(uc));
		let min = ua.min(ub.min(uc));
		let offset = 0.5 * (max + min);

		ua -= offset;
		ub -= offset;
		uc -= offset;

		(ua, ub, uc)
	}

	/// 测量dq
	///
	#[inline]
	pub fn measure_dq_by_mech_pos(
		ia: f32,
		ib: f32,
		ic: f32,
		theta_mech: f32,
		pole_pairs: u8,
	) -> (f32, f32) {
		// Clarke
		let (alpha, beta) = Self::clarke(ia, ib, ic);
		// Park
		let (id, iq) = Self::park(
			alpha,
			beta,
			theta_mech * (pole_pairs as f32),
		);
		(id, iq)
	}

	#[inline]
	pub fn measure_dq(
		ia: f32,
		ib: f32,
		ic: f32,
		theta: f32,
	) -> (f32, f32) {
		// Clarke
		let (alpha, beta) = Self::clarke(ia, ib, ic);
		// Park
		let (id, iq) = Self::park(
			alpha,
			beta,
			theta,
		);
		(id, iq)
	}
}
