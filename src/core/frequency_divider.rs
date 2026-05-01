#[derive(Clone, Copy)]
pub struct FrequencyDivider {
	pub div: u32,
	cnt: u32,
}

impl FrequencyDivider {
	pub fn new(div: u32) -> Self {
		Self { div, cnt: 0 }
	}

	#[inline(always)]
	pub fn tick(&mut self) -> bool {
		self.cnt += 1;
		if self.cnt >= self.div {
			self.cnt = 0;
			true
		} else {
			false
		}
	}
}
