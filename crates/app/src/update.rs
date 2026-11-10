//! Updates: a background thread asks GitHub Releases for a newer version when the game starts and
//! every 10 minutes after, and downloads it. Once one is downloaded, a button offers to restart
//! into it now; otherwise Velopack applies it the next time the game starts.

use bevy::prelude::*;
use bevy::ui_widgets::{Activate, Button};
use std::sync::{Arc, Mutex};
use std::time::Duration;

const REPO: &str = "https://github.com/TylerHandel/rebuilt-sim-native";
const RECHECK: Duration = Duration::from_secs(10 * 60);

#[derive(Default)]
struct State { text: String, ready: Option<velopack::UpdateInfo> }

#[derive(Resource, Clone, Default)]
pub struct UpdateStatus(Arc<Mutex<State>>);

impl UpdateStatus {
    pub fn text(&self) -> String { self.0.lock().map(|s| s.text.clone()).unwrap_or_default() }
    fn set(&self, s: impl Into<String>) { if let Ok(mut g) = self.0.lock() { g.text = s.into(); } }
    fn ready_version(&self) -> Option<String> { self.0.lock().ok()?.ready.as_ref().map(|i| i.TargetFullRelease.Version.clone()) }
}

pub struct UpdatePlugin;

impl Plugin for UpdatePlugin {
    fn build(&self, app: &mut App) {
        let status = UpdateStatus::default();
        let s = status.clone();
        std::thread::spawn(move || watch(&s));
        app.insert_resource(status)
            .add_systems(Startup, spawn_button)
            .add_systems(Update, show_button);
    }
}

fn manager() -> Option<velopack::UpdateManager> {
    velopack::UpdateManager::new(velopack::sources::GithubSource::new(REPO, None, false), None, None).ok()
}

/// checks now and every RECHECK, until one is downloaded
fn watch(status: &UpdateStatus) {
    use velopack::UpdateCheck;
    // (not installed, a `cargo run`: nothing to update. REBUILT_FAKE_UPDATE=<version> pretends one
    // is ready, to see the button)
    let Some(um) = manager() else {
        if let Ok(v) = std::env::var("REBUILT_FAKE_UPDATE") {
            let mut info = velopack::UpdateInfo::default();
            info.TargetFullRelease.Version = v;
            if let Ok(mut g) = status.0.lock() { g.ready = Some(info); }
        }
        return status.set("development build (no updates)");
    };
    let v = um.get_current_version_as_string();
    loop {
        status.set(format!("v{v} | checking for updates..."));
        match um.check_for_updates() {
            Ok(UpdateCheck::UpdateAvailable(info)) => {
                let to = info.TargetFullRelease.Version.clone();
                status.set(format!("v{v} | downloading v{to}..."));
                match um.download_updates(&info, None) {
                    Ok(()) => {
                        status.set(format!("v{v} | v{to} is ready: restart to play it (or it installs the next time you start the game)"));
                        if let Ok(mut g) = status.0.lock() { g.ready = Some(*info); }
                        return;
                    }
                    Err(e) => status.set(format!("v{v} | download failed: {e}")),
                }
            }
            Ok(_) => status.set(format!("v{v} | up to date")),
            Err(e) => status.set(format!("v{v} | update check failed: {e}")),
        }
        std::thread::sleep(RECHECK);
    }
}

#[derive(Component)]
struct RestartButton;
#[derive(Component)]
struct RestartLabel;

fn spawn_button(mut commands: Commands) {
    commands.spawn((
        Button,
        Node {
            position_type: PositionType::Absolute, top: Val::Px(10.0), right: Val::Px(12.0),
            padding: UiRect::axes(Val::Px(14.0), Val::Px(8.0)), display: Display::None,
            ..default()
        },
        BackgroundColor(Color::srgb(0.13, 0.55, 0.25)),
        RestartButton,
    )).observe(restart).with_children(|b| {
        b.spawn((Text::new("Restart to update"), TextFont { font_size: FontSize::Px(16.0), ..default() }, TextColor(Color::WHITE), RestartLabel));
    });
}

fn show_button(status: Res<UpdateStatus>, mut node: Query<&mut Node, With<RestartButton>>, mut label: Query<&mut Text, With<RestartLabel>>) {
    let Some(v) = status.ready_version() else { return };
    if let Ok(mut n) = node.single_mut() { if n.display == Display::None { n.display = Display::Flex; } }
    if let Ok(mut t) = label.single_mut() { let want = format!("Restart to update to v{v}"); if t.0 != want { t.0 = want; } }
}

/// the button's click: exit the game, apply the update, start the new version
fn restart(_: On<Activate>, status: Res<UpdateStatus>) {
    let info = status.0.lock().ok().and_then(|g| g.ready.clone());
    if let (Some(info), Some(um)) = (info, manager()) {
        status.set("restarting into the update...");
        if let Err(e) = um.apply_updates_and_restart(&info.TargetFullRelease) {
            status.set(format!("update failed: {e}"));
        }
    }
}
