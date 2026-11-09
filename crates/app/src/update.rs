//! Updates: on start, a background thread asks GitHub Releases for a newer version, downloads it,
//! and Velopack applies it the next time the game starts.

use bevy::prelude::*;
use std::sync::{Arc, Mutex};

const REPO: &str = "https://github.com/TylerHandel/rebuilt-sim-native";

#[derive(Resource, Clone, Default)]
pub struct UpdateStatus(Arc<Mutex<String>>);

impl UpdateStatus {
    pub fn text(&self) -> String {
        self.0.lock().map(|s| s.clone()).unwrap_or_default()
    }
    fn set(&self, s: impl Into<String>) {
        if let Ok(mut g) = self.0.lock() { *g = s.into(); }
    }
}

pub struct UpdatePlugin;

impl Plugin for UpdatePlugin {
    fn build(&self, app: &mut App) {
        let status = UpdateStatus::default();
        let s = status.clone();
        std::thread::spawn(move || check(&s));
        app.insert_resource(status);
    }
}

fn check(status: &UpdateStatus) {
    use velopack::{sources::GithubSource, UpdateCheck, UpdateManager};
    let um = match UpdateManager::new(GithubSource::new(REPO, None, false), None, None) {
        Ok(um) => um,
        // not installed (a `cargo run`): nothing to update
        Err(_) => return status.set("development build (no updates)"),
    };
    let v = um.get_current_version_as_string();
    status.set(format!("v{v} | checking for updates..."));
    match um.check_for_updates() {
        Ok(UpdateCheck::UpdateAvailable(info)) => {
            let to = info.TargetFullRelease.Version.clone();
            status.set(format!("v{v} | downloading v{to}..."));
            match um.download_updates(&info, None) {
                // (VelopackApp applies a downloaded update the next time the game starts)
                Ok(()) => status.set(format!("v{v} | v{to} downloaded: it installs the next time you start the game")),
                Err(e) => status.set(format!("v{v} | download failed: {e}")),
            }
        }
        Ok(_) => status.set(format!("v{v} | up to date")),
        Err(e) => status.set(format!("v{v} | update check failed: {e}")),
    }
}
