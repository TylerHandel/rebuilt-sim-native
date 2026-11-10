//! Updates: a background thread asks GitHub Releases for a newer version when the game starts and
//! every minute after, and downloads it. Once one is downloaded the game restarts into it by itself
//! as soon as no match is being played (after a few seconds' notice, which a click skips); during a
//! match a button offers to restart now. Closing the game also installs it (Velopack applies a
//! downloaded update at the next start).

use bevy::prelude::*;
use bevy::ui_widgets::{Activate, Button};
use std::sync::{Arc, Mutex};
use std::time::Duration;

const REPO: &str = "https://github.com/TylerHandel/rebuilt-sim-native";
const RECHECK: Duration = Duration::from_secs(60);
/// The notice before restarting by itself, s.
const NOTICE: f32 = 5.0;

/// Whether a match is being played (the game sets it; an update waits for the match to end).
#[derive(Resource, Default)]
pub struct InMatch(pub bool);

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
            .init_resource::<InMatch>()
            .add_systems(Startup, spawn_button)
            .add_systems(Update, show_button);
    }
}

/// The latest release's files, straight from its download links: no GitHub API calls, so checking
/// every minute stays clear of the API's hourly limit (60 per address, shared by every PC on a
/// network). Its feed lists the latest version only; more than one version behind, the delta
/// doesn't apply and Velopack downloads the full package.
fn manager() -> Option<velopack::UpdateManager> {
    let source = velopack::sources::HttpSource::new(format!("{REPO}/releases/latest/download"));
    velopack::UpdateManager::new(source, None, None).ok()
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
                        status.set(format!("v{v} | v{to} is ready: it installs when no match is on"));
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

/// Shows the button once an update is ready: during a match, an offer to restart now; otherwise a
/// countdown, then the restart.
fn show_button(
    status: Res<UpdateStatus>,
    in_match: Res<InMatch>,
    time: Res<Time>,
    mut left: Local<Option<f32>>,
    mut node: Query<&mut Node, With<RestartButton>>,
    mut label: Query<&mut Text, With<RestartLabel>>,
) {
    let Some(v) = status.ready_version() else { return };
    if let Ok(mut n) = node.single_mut() { if n.display == Display::None { n.display = Display::Flex; } }
    let want = if in_match.0 {
        *left = None;
        format!("Restart to update to v{v}")
    } else {
        let t = left.get_or_insert(NOTICE);
        if *t <= 0.0 { return }
        *t -= time.delta_secs();
        if *t <= 0.0 {
            apply(&status);
            format!("Updating to v{v}...")
        } else {
            format!("Updating to v{v} in {}... (click to restart now)", t.ceil() as i32)
        }
    };
    if let Ok(mut t) = label.single_mut() { if t.0 != want { t.0 = want; } }
}

/// the button's click: restart now
fn restart(_: On<Activate>, status: Res<UpdateStatus>) {
    apply(&status);
}

/// exits the game, applies the update and starts the new version (returns only if that fails)
fn apply(status: &UpdateStatus) {
    let info = status.0.lock().ok().and_then(|g| g.ready.clone());
    if let (Some(info), Some(um)) = (info, manager()) {
        status.set("restarting into the update...");
        if let Err(e) = um.apply_updates_and_restart(&info.TargetFullRelease) {
            status.set(format!("update failed: {e}"));
        }
    }
}
