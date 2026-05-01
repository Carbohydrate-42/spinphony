pub trait DynamicSystem {
	type Param;
	type State;
	type Input;
	type Output;

	fn new(param: Self::Param) -> Self;
	// fn reset(&mut self);
	fn set_input(&mut self, input: Self::Input);
	fn update(&mut self, dt: f32);
	fn output(&self) -> Self::Output;
}
