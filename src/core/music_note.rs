use crate::constant::midi_freq::MIDI_FREQ;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Note {
	pub freq: f32,
	pub duration: f32, // 秒
}

/// 标准 MIDI 频率计算
/// A4 = 440Hz
/// MIDI 69 = A4
pub const fn note_freq(oct: i32, semi: i32) -> f32 {
	let midi = (oct + 1) * 12 + semi;

	// 边界保护（防止 UB）
	if midi < 0 || midi >= 128 {
		0.0
	} else {
		MIDI_FREQ[midi as usize]
	}
}

#[macro_export]
macro_rules! note {
    (C  $oct:expr) => { $crate::core::music_note::note_freq($oct, 0) };
    (CS $oct:expr) => { $crate::core::music_note::note_freq($oct, 1) };

    (D  $oct:expr) => { $crate::core::music_note::note_freq($oct, 2) };
    (DS $oct:expr) => { $crate::core::music_note::note_freq($oct, 3) };

    (E  $oct:expr) => { $crate::core::music_note::note_freq($oct, 4) };

    (F  $oct:expr) => { $crate::core::music_note::note_freq($oct, 5) };
    (FS $oct:expr) => { $crate::core::music_note::note_freq($oct, 6) };

    (G  $oct:expr) => { $crate::core::music_note::note_freq($oct, 7) };
    (GS $oct:expr) => { $crate::core::music_note::note_freq($oct, 8) };

    (A  $oct:expr) => { $crate::core::music_note::note_freq($oct, 9) };
    (AS $oct:expr) => { $crate::core::music_note::note_freq($oct, 10) };

    (B  $oct:expr) => { $crate::core::music_note::note_freq($oct, 11) };
}


// #[cfg(test)]
// mod tests {
// 	use super::*;
//
// 	#[test]
// 	fn test_note_freq_matches_standard() {
// 		assert!((note!(G 3) - 196.00).abs() < 0.01);
// 		assert!((note!(A 3) - 220.00).abs() < 0.01);
// 		assert!((note!(B 3) - 246.94).abs() < 0.01);
// 		assert!((note!(C 4) - 261.63).abs() < 0.01);
// 		assert!((note!(FS 4) - 369.99).abs() < 0.01);
// 		assert!((note!(E 5) - 659.25).abs() < 0.01);
// 	}
// }
