use crate::core::music_note::Note;
use crate::note;

pub const STANDARD_TEST_NOTE: [Note; 8] = [
	Note { freq: note!(C 4), duration: 2.0 },
	Note { freq: note!(D 4), duration: 2.0 },
	Note { freq: note!(E 4), duration: 2.0 },
	Note { freq: note!(F 4), duration: 2.0 },
	Note { freq: note!(G 4), duration: 2.0 },
	Note { freq: note!(A 4), duration: 2.0 },
	Note { freq: note!(B 4), duration: 2.0 },
	Note { freq: note!(C 5), duration: 2.0 },
];
