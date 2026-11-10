//! The browser game's UI (index.html, css/style.css, js/ui.js) in Bevy UI: the menus, the match
//! HUD, pause and results. Its colors, sizes and fonts are the stylesheet's.

pub mod hud;
pub mod menu;
pub mod overlays;

use bevy::prelude::*;

/// Which screen is up (js ui.show(screen))
#[derive(States, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Screen {
    #[default]
    Menu,
    Playing,
    Paused,
    Results,
}

/// css :root
pub mod color {
    use bevy::prelude::Color;
    pub const BLUE: Color = Color::srgb(0x2f as f32 / 255.0, 0x6f as f32 / 255.0, 0xf0 as f32 / 255.0);
    pub const BLUE_D: Color = Color::srgb(0x17 as f32 / 255.0, 0x3a as f32 / 255.0, 0x8a as f32 / 255.0);
    pub const RED: Color = Color::srgb(0xe0 as f32 / 255.0, 0x34 as f32 / 255.0, 0x34 as f32 / 255.0);
    pub const RED_D: Color = Color::srgb(0x7d as f32 / 255.0, 0x17 as f32 / 255.0, 0x17 as f32 / 255.0);
    pub const FUEL: Color = Color::srgb(0xf4 as f32 / 255.0, 0xd2 as f32 / 255.0, 0x3a as f32 / 255.0);
    /// (css .86; Bevy blends in linear light, where the same alpha lets more of a bright field through)
    pub const BG: Color = Color::srgba(12.0 / 255.0, 14.0 / 255.0, 20.0 / 255.0, 0.95);
    pub const PANEL: Color = Color::srgba(14.0 / 255.0, 17.0 / 255.0, 24.0 / 255.0, 0.78);
    /// (css rgba(255,255,255,.12) as the browser blends it over the dark menus and panels: see over)
    pub const LINE: Color = Color::srgb(41.0 / 255.0, 43.0 / 255.0, 48.0 / 255.0);
    pub const TEXT: Color = Color::srgb(0xee as f32 / 255.0, 0xf1 as f32 / 255.0, 0xf6 as f32 / 255.0);
    pub const MUTED: Color = Color::srgb(0x9a as f32 / 255.0, 0xa3 as f32 / 255.0, 0xb2 as f32 / 255.0);
    pub const OK: Color = Color::srgb(0x37 as f32 / 255.0, 0xd6 as f32 / 255.0, 0x7a as f32 / 255.0);
    pub const WARN: Color = Color::srgb(1.0, 0xb0 as f32 / 255.0, 0x20 as f32 / 255.0);
    pub const BAD: Color = Color::srgb(1.0, 0x6b as f32 / 255.0, 0x6b as f32 / 255.0);
    pub const DARK: Color = Color::srgb(0x11 as f32 / 255.0, 0x11 as f32 / 255.0, 0x11 as f32 / 255.0);
    pub fn white(a: f32) -> Color { Color::srgba(1.0, 1.0, 1.0, a) }
    /// A css rgba(r, g, b, a) over the menus' dark background (#0c0e14), mixed as the browser mixes
    /// it (in sRGB), as an opaque color: Bevy blends translucent UI in linear light, where a faint
    /// tint over a dark background comes out about 2.5 times as bright
    pub fn over(r: u8, g: u8, b: u8, a: f32) -> Color {
        let m = |c: u8, k: f32| (c as f32 * a + k * (1.0 - a)) / 255.0;
        Color::srgb(m(r, 12.0), m(g, 14.0), m(b, 20.0))
    }
    /// over() for white
    pub fn tint(a: f32) -> Color { over(255, 255, 255, a) }
    pub fn rgba(r: u8, g: u8, b: u8, a: f32) -> Color { Color::srgba(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0, a) }
}

/// The stylesheet's two faces: Barlow Condensed (headings, numbers, buttons) and Inter (the rest)
#[derive(Resource, Clone)]
pub struct Fonts {
    pub barlow: [Handle<Font>; 3],
    pub inter: Handle<Font>,
}

/// A font face and weight
#[derive(Clone, Copy)]
pub enum Face {
    /// Barlow Condensed 600 / 700 / 800
    Barlow(u16),
    /// Inter, any weight
    Inter(u16),
}

impl Fonts {
    pub fn font(&self, face: Face, size: f32) -> TextFont {
        match face {
            Face::Barlow(w) => {
                let i = if w >= 800 { 2 } else if w >= 700 { 1 } else { 0 };
                TextFont { font: self.barlow[i].clone().into(), font_size: FontSize::Px(size), ..default() }
            }
            Face::Inter(w) => TextFont { font: self.inter.clone().into(), font_size: FontSize::Px(size), weight: FontWeight(w), ..default() },
        }
    }

    /// A text node
    pub fn text(&self, s: impl Into<String>, face: Face, size: f32, color: Color) -> impl Bundle {
        (Text::new(s), self.font(face, size), TextColor(color))
    }
}

pub fn radius(px: f32) -> BorderRadius { BorderRadius::all(Val::Px(px)) }

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<Screen>()
            .add_systems(PreStartup, load_fonts)
            .add_plugins((menu::MenuPlugin, hud::HudPlugin, overlays::OverlaysPlugin));
    }
}

fn load_fonts(mut commands: Commands, assets: Res<AssetServer>) {
    commands.insert_resource(Fonts {
        barlow: [
            assets.load("fonts/BarlowCondensed-SemiBold.ttf"),
            assets.load("fonts/BarlowCondensed-Bold.ttf"),
            assets.load("fonts/BarlowCondensed-ExtraBold.ttf"),
        ],
        inter: assets.load("fonts/Inter.ttf"),
    });
}

/// Menu-style input from the keyboard and every controller (js input.js nav and pressed): one step
/// per press, repeating while held
#[derive(Default, Clone, Copy)]
pub struct Nav { pub up: bool, pub down: bool, pub left: bool, pub right: bool, pub ok: bool, pub back: bool }

pub fn read_nav(keys: &ButtonInput<KeyCode>, pads: &Query<&Gamepad>, held: &mut NavRepeat, dt: f32) -> Nav {
    let k = |c: &[KeyCode]| c.iter().any(|c| keys.just_pressed(*c));
    let kh = |c: &[KeyCode]| c.iter().any(|c| keys.pressed(*c));
    let mut n = Nav {
        ok: k(&[KeyCode::Enter, KeyCode::NumpadEnter, KeyCode::Space]),
        back: k(&[KeyCode::Escape, KeyCode::Backspace]),
        ..default()
    };
    let mut dirs = [
        kh(&[KeyCode::ArrowUp, KeyCode::KeyW]), kh(&[KeyCode::ArrowDown, KeyCode::KeyS]),
        kh(&[KeyCode::ArrowLeft, KeyCode::KeyA]), kh(&[KeyCode::ArrowRight, KeyCode::KeyD]),
    ];
    for p in pads {
        n.ok |= p.just_pressed(GamepadButton::South);
        n.back |= p.just_pressed(GamepadButton::East);
        let (x, y) = (p.get(GamepadAxis::LeftStickX).unwrap_or(0.0), p.get(GamepadAxis::LeftStickY).unwrap_or(0.0));
        dirs[0] |= p.pressed(GamepadButton::DPadUp) || y > 0.6;
        dirs[1] |= p.pressed(GamepadButton::DPadDown) || y < -0.6;
        dirs[2] |= p.pressed(GamepadButton::DPadLeft) || x < -0.6;
        dirs[3] |= p.pressed(GamepadButton::DPadRight) || x > 0.6;
    }
    let mut fire = [false; 4];
    for i in 0..4 {
        if dirs[i] {
            held.t[i] += dt;
            // (a press, then repeats after 0.4 s every 0.09 s)
            if held.t[i] == dt || (held.t[i] > 0.4 && held.t[i] - held.last[i] > 0.09) {
                fire[i] = true;
                if held.t[i] > 0.4 { held.last[i] = held.t[i]; }
            }
        } else {
            held.t[i] = 0.0;
            held.last[i] = 0.0;
        }
    }
    [n.up, n.down, n.left, n.right] = fire;
    n
}

#[derive(Default)]
pub struct NavRepeat { t: [f32; 4], last: [f32; 4] }
