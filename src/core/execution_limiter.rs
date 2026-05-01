#[derive(Clone, Copy, Debug)]
pub struct ExecutionLimiter {
	remaining: u32, // 剩余可执行次数
}

impl ExecutionLimiter {
	/// 创建一个新的 ExecutionLimiter，允许执行 `n` 次
	pub fn new(n: u32) -> Self {
		Self { remaining: n }
	}

	/// 每次调用 tick，如果还有剩余次数就返回 true，并减少一次
	/// 否则返回 false 永久不再触发
	#[inline(always)]
	pub fn tick(&mut self) -> bool {
		if self.remaining > 0 {
			self.remaining -= 1;
			true
		} else {
			false
		}
	}

	/// 获取剩余可执行次数
	pub fn remaining(&self) -> u32 {
		self.remaining
	}
}
