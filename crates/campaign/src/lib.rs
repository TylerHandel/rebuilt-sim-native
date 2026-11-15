//! Road to Einstein, the campaign (docs/CAMPAIGN.md): its events and season as pure logic, tested
//! headless. The game plays your matches; every other match at an event is decided here from the
//! robots' ratings (assets/ratings.json) and their drivers' skills, with luck.

pub mod event;
pub mod model;
pub mod rng;
pub mod season;

pub use event::{Event, Level, Line, Rules};
pub use model::{Ratings, Skill, Team};
