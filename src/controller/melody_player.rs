use crate::core::music_note::Note;
use defmt::debug;

pub struct MelodyPlayer<'a> {
	notes: &'a [Note],
	index: usize,
	time_left: f32,
	finished: bool,
}

impl<'a> MelodyPlayer<'a> {
	pub fn new(notes: &'a [Note]) -> Self {
		let first_duration = if !notes.is_empty() {
			notes[0].duration
		} else {
			0.0
		};

		Self {
			notes,
			index: 0,
			time_left: first_duration,
			finished: notes.is_empty(),
		}
	}

	pub fn update(&mut self, dt: f32) {
		if self.finished {
			return;
		}

		self.time_left -= dt;

		if self.time_left <= 0.0 {
			self.index += 1;

			if self.index >= self.notes.len() {
				self.finished = true;
			} else {
				debug!(
					"Playing note : frequency {} duration {}",
					self.notes[self.index].freq,
					self.notes[self.index].duration
				);
				self.time_left = self.notes[self.index].duration;
			}
		}
	}

	pub fn current_freq(&self) -> f32 {
		if self.finished {
			0.0
		} else {
			self.notes[self.index].freq
		}
	}

	pub fn is_playing(&self) -> bool {
		!self.finished
	}
}
