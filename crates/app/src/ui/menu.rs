//! The menus (js ui.js PAGES, renderMenu; css #menu): the home page's mode tiles and buttons, the
//! Practice page (the robot picker and the match options) and Settings. Each page is a list of
//! items, rebuilt whenever anything changes; the keyboard, a controller (D-pad or stick to move,
//! left / right to change, A to choose, B to go back) and the mouse (hover focuses, click acts)
//! all work it. 1 v 1 and 3 v 3 need the AI drivers, which this version doesn't have yet.

use bevy::prelude::*;
use bevy::ui::ComputedNode;

use super::{color, radius, read_nav, Face, Fonts, NavRepeat, Screen};
use crate::game::GameCmd;
use crate::settings::{Roster, Settings};
use crate::update::UpdateStatus;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Page { Home, Practice, Settings }

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Mode { Practice, OneVOne, ThreeVThree }
const MODES: [Mode; 3] = [Mode::Practice, Mode::OneVOne, Mode::ThreeVThree];

impl Mode {
    /// js ui.js MODES
    fn name(self) -> &'static str { match self { Mode::Practice => "Practice", Mode::OneVOne => "1 v 1", Mode::ThreeVThree => "3 v 3" } }
    fn line(self) -> &'static str { match self { Mode::Practice => "Just your robot on the field", Mode::OneVOne => "You against an AI robot", Mode::ThreeVThree => "Full alliances" } }
    fn desc(self) -> &'static str {
        match self {
            Mode::Practice => "Drive, collect and shoot with the field to yourself: learn a robot, try autos, work on your cycles.",
            Mode::OneVOne => "You against one AI-driven robot on the other ALLIANCE. Not in this version yet: it needs the AI drivers, which come over after the robots' mechanisms.",
            Mode::ThreeVThree => "Two full ALLIANCES of three. Not in this version yet: it needs the AI drivers and the rules between robots, which come over after the robots' mechanisms.",
        }
    }
    fn ready(self) -> bool { self == Mode::Practice }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OptId { Auto, Start, Alliance, Ds, Hp, Camera, Graphics, Motion, Fps, Venue, Time, Weather, Haze, Film }

#[derive(Clone, Copy, PartialEq, Debug)]
enum Item {
    Section(&'static str),
    Tiles,
    Robot,
    Opt(OptId),
    Btn { label: &'static str, act: Act, secondary: bool, half: bool, foot: bool },
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Act { Open(Page), Start, Controls, Quit, Tile(Mode), StepRobot(i32), SetRobot(usize), Step(OptId, i32), Set(OptId, usize) }

/// The menu's state (js UI: page, focus, tile)
#[derive(Resource)]
pub struct Menu {
    pub page: Page,
    pub focus: usize,
    tile: Mode,
    pub dirty: bool,
}

impl Default for Menu { fn default() -> Self { Menu { page: Page::Home, focus: 0, tile: Mode::Practice, dirty: true } } }

/// The Controls screen is open (over the menu or the pause menu)
#[derive(Resource, Default)]
pub struct ControlsOpen(pub bool);

fn items(page: Page) -> Vec<Item> {
    let btn = |label, act, secondary, half, foot| Item::Btn { label, act, secondary, half, foot };
    match page {
        Page::Home => vec![
            Item::Tiles,
            btn("CONTROLS", Act::Controls, true, true, false),
            btn("SETTINGS", Act::Open(Page::Settings), true, true, false),
            btn("QUIT", Act::Quit, true, false, false),
        ],
        Page::Practice => vec![
            Item::Section("Robot"), Item::Robot,
            Item::Section("Match"), Item::Opt(OptId::Auto), Item::Opt(OptId::Start), Item::Opt(OptId::Alliance), Item::Opt(OptId::Ds),
            Item::Section("Your robot"), Item::Opt(OptId::Hp),
            btn("START MATCH", Act::Start, false, false, true),
            btn("BACK", Act::Open(Page::Home), true, false, true),
        ],
        Page::Settings => vec![
            Item::Section("View"), Item::Opt(OptId::Camera),
            Item::Section("Graphics"), Item::Opt(OptId::Graphics), Item::Opt(OptId::Motion), Item::Opt(OptId::Fps),
            Item::Section("Atmosphere"), Item::Opt(OptId::Venue), Item::Opt(OptId::Time), Item::Opt(OptId::Weather), Item::Opt(OptId::Haze), Item::Opt(OptId::Film),
            btn("BACK", Act::Open(Page::Home), true, false, true),
        ],
    }
}

fn focusables(page: Page) -> Vec<Item> { items(page).into_iter().filter(|i| !matches!(i, Item::Section(_))).collect() }

// ------------------------------------------------------------------ options (js ui.js OPT)
fn opt_label(id: OptId) -> &'static str {
    match id {
        OptId::Auto => "Auto routine", OptId::Start => "Starting position", OptId::Alliance => "Alliance", OptId::Ds => "Driver station", OptId::Hp => "Your human player",
        OptId::Camera => "Starting camera", OptId::Graphics => "Graphics", OptId::Motion => "Motion effects", OptId::Fps => "Show FPS",
        OptId::Venue => "Venue", OptId::Time => "Time of day", OptId::Weather => "Weather", OptId::Haze => "Volumetric fog", OptId::Film => "Film look",
    }
}

fn opt_values(id: OptId) -> Vec<(String, String)> {
    let v = |a: &str, b: &str| (a.to_string(), b.to_string());
    match id {
        OptId::Auto => vec![v("drive", "Drive AUTO yourself"), v("skip", "Skip AUTO"), v("none", "Stand still in AUTO")],
        OptId::Start => Roster::get().starts.iter().map(|(k, n, _)| (k.clone(), n.clone())).collect(),
        OptId::Alliance => vec![v("blue", "Blue"), v("red", "Red")],
        OptId::Ds => vec![v("0", "Left (DS1)"), v("1", "Center (DS2)"), v("2", "Right (DS3)")],
        OptId::Hp => vec![v("auto", "Auto-throw when HUB active"), v("manual", "Manual (X / Y)")],
        OptId::Camera => crate::camera::CAMERA_MODES.iter().map(|m| v(m.key(), m.name())).collect(),
        OptId::Graphics => vec![v("ultra", "Ultra"), v("high", "High"), v("medium", "Medium"), v("low", "Low")],
        OptId::Motion => vec![v("off", "Off"), v("trails", "Trails"), v("blur", "Blur"), v("both", "Both")],
        OptId::Fps => vec![v("off", "Off"), v("on", "On")],
        OptId::Venue => vec![v("arena", "Arena"), v("stadium", "Stadium")],
        OptId::Time => vec![v("day", "Day"), v("sunset", "Sunset"), v("night", "Night")],
        OptId::Weather => vec![v("clear", "Clear"), v("cloudy", "Cloudy"), v("rain", "Rain"), v("fog", "Fog")],
        OptId::Haze => vec![v("off", "Off"), v("low", "Low"), v("high", "High")],
        OptId::Film => vec![v("off", "Off"), v("on", "On")],
    }
}

fn opt_get(id: OptId, s: &Settings) -> String {
    match id {
        OptId::Auto => s.auto.clone(), OptId::Start => s.start.clone(), OptId::Alliance => s.alliance.clone(),
        OptId::Ds => s.ds.to_string(), OptId::Hp => s.hp.clone(), OptId::Camera => s.camera.clone(),
        OptId::Graphics => s.graphics.clone(), OptId::Motion => s.motion.clone(), OptId::Fps => if s.fps { "on".into() } else { "off".into() },
        OptId::Venue => s.venue.clone(), OptId::Time => s.time.clone(), OptId::Weather => s.weather.clone(), OptId::Haze => s.haze.clone(),
        OptId::Film => if s.film { "on".into() } else { "off".into() },
    }
}

fn opt_set(id: OptId, s: &mut Settings, v: &str) {
    match id {
        OptId::Auto => s.auto = v.into(), OptId::Start => s.start = v.into(), OptId::Alliance => s.alliance = v.into(),
        OptId::Ds => s.ds = v.parse().unwrap_or(1), OptId::Hp => s.hp = v.into(), OptId::Camera => s.camera = v.into(),
        OptId::Graphics => s.graphics = v.into(), OptId::Motion => s.motion = v.into(), OptId::Fps => s.fps = v == "on",
        OptId::Venue => s.venue = v.into(), OptId::Time => s.time = v.into(), OptId::Weather => s.weather = v.into(), OptId::Haze => s.haze = v.into(),
        OptId::Film => s.film = v == "on",
    }
}

fn opt_desc(id: OptId, s: &Settings) -> String {
    match id {
        OptId::Auto => match s.auto.as_str() {
            "drive" => "Drive AUTO yourself: the 20 s AUTO runs with you at the controls, as practice for writing one (in a real match drivers can't control the robot in AUTO, G402). FUEL scored counts as AUTO's.".into(),
            "skip" => "Skip: GET READY, then straight to TELEOP (who goes inactive first is decided as after an AUTO with no FUEL scored). The teams' own AUTOs and the built-in routines aren't in this version yet.".into(),
            _ => "Stand still: the 20 s AUTO runs with the robot waiting (drivers can't control the robot in AUTO, G402). The teams' own AUTOs and the built-in routines aren't in this version yet.".into(),
        },
        OptId::Start => "BUMPERS overlap the ROBOT STARTING LINE without touching a BUMP (G303).".into(),
        OptId::Alliance => "Which ALLIANCE you drive for: you start on its side of the field.".into(),
        OptId::Ds => "Where you stand behind your ALLIANCE WALL (for the Driver Station camera).".into(),
        OptId::Hp => "The human player at your OUTPOST: it throws FUEL by itself whenever your HUB is active, or you throw it yourself (Y / H held; X / G opens the CHUTE DOOR).".into(),
        OptId::Camera => "Change it any time in a match with D-pad ← → ([ / ]); right stick click (T) turns it around.".into(),
        OptId::Graphics => match s.graphics.as_str() {
            "ultra" => "Ultra: High, plus temporal anti-aliasing (smooth edges, no shimmer on thin parts) and sharp shadows out to the far wall. For a gaming GPU.",
            "high" => "High: Medium, plus big shadows, 4x anti-aliasing, a soft vignette, and glowing HUB lights, robot LEDs and truss lights (bloom).",
            "medium" => "Medium: the arena (its crowd, trusses and screens), its lights and reflections, shadows and FXAA edges.",
            _ => "Low: no arena, shadows or anti-aliasing, for slower PCs.",
        }.into(),
        OptId::Motion => match s.motion.as_str() {
            "trails" => "Trails: a fading streak behind each shot FUEL.",
            "blur" => "Blur: motion blur on what moves fast (the camera, shots, spinning wheels).",
            "both" => "Both: shot trails and motion blur.",
            _ => "Off: no trails or motion blur.",
        }.into(),
        OptId::Fps => "Frame rate, physics time per step and the update status in the corner.".into(),
        OptId::Venue => if s.venue == "stadium" { "Stadium: outdoors, under the sky, its stands round the field and light towers over them; the weather's yours to pick." } else { "Arena: indoors, in a gym, the crowd in its bleachers, the lights on the trusses and in the roof." }.into(),
        OptId::Time => match s.time.as_str() {
            "sunset" => "Sunset: a low, warm sun and long shadows (indoors: an evening, the house lights half down).",
            "night" => "Night: the house lights down and the show's lights up: moving colored spots, the robots' LEDs lighting the carpet, the crowd's camera flashes (outdoors: under the light towers and the stars). Without the sun's shadows it runs faster than day.",
            _ => "Day: the sun high (indoors: the gym's lights all on).",
        }.into(),
        OptId::Weather => match s.weather.as_str() {
            "cloudy" => "Cloudy: clouds across the sky, the sun in and out of them. Outdoors only.",
            "rain" => "Rain: falling rain, an overcast sky, everything wet and glossy, a haze in the distance. Outdoors only.",
            "fog" => "Fog: the stands fading into it, the lights glowing through it. Outdoors only.",
            _ => "Clear: a clear sky. Outdoors only.",
        }.into(),
        OptId::Haze => match s.haze.as_str() {
            "low" => "Low: the lights' beams in a light haze over the field (24 steps a pixel). A few ms on a mid-range GPU; scales with resolution.",
            "high" => "High: a thicker haze, smoother beams, and the sun's rays too (48 steps a pixel). For a strong GPU.",
            _ => "Off: clear air.",
        }.into(),
        OptId::Film => "Film look: filmic tone mapping (rich colors, soft highlights), a touch of lens fringing at the edges, fine film grain, and eyes that adapt between bright and dark.".into(),
    }
}

/// js ui.js _segmented: a few short values side by side; longer lists get a stepper
fn segmented(id: OptId) -> bool {
    let vals = opt_values(id);
    vals.len() <= 4 && vals.iter().map(|v| v.1.chars().count() + 3).sum::<usize>() <= 30
}

fn cycle(id: OptId, s: &mut Settings, dir: i32) {
    let vals = opt_values(id);
    let cur = opt_get(id, s);
    let i = vals.iter().position(|v| v.0 == cur).unwrap_or(0) as i32;
    let n = vals.len() as i32;
    let j = ((i + dir) % n + n) % n;
    opt_set(id, s, &vals[j as usize].0);
}

// ------------------------------------------------------------------ plugin
pub struct MenuPlugin;

impl Plugin for MenuPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Menu>()
            .init_resource::<ControlsOpen>()
            .init_resource::<Acts>()
            .add_systems(OnEnter(Screen::Menu), |mut m: ResMut<Menu>| m.dirty = true)
            .add_systems(OnExit(Screen::Menu), despawn_menu)
            .add_systems(Update, (input, apply_acts, rebuild, center_strip, foot_status).chain().run_if(in_state(Screen::Menu)));
    }
}

#[derive(Component)]
struct MenuRoot;
#[derive(Component)]
struct Focus(usize);
#[derive(Component)]
struct Click(Act);
#[derive(Component)]
struct Strip(usize);
#[derive(Component)]
struct FootStatus;

/// What the mouse asked for this frame
#[derive(Resource, Default)]
struct Acts(Vec<Act>);

fn despawn_menu(mut commands: Commands, q: Query<Entity, With<MenuRoot>>) {
    for e in &q { commands.entity(e).despawn(); }
}

fn input(
    keys: Res<ButtonInput<KeyCode>>,
    pads: Query<&Gamepad>,
    time: Res<Time>,
    mut rep: Local<NavRepeat>,
    mut menu: ResMut<Menu>,
    mut settings: ResMut<Settings>,
    mut acts: ResMut<Acts>,
    controls: Res<ControlsOpen>,
) {
    let nav = read_nav(&keys, &pads, &mut rep, time.delta_secs());
    // (and not on the press that closed the Controls)
    if controls.0 || controls.is_changed() { return }
    let f = focusables(menu.page);
    let n = f.len();
    if nav.up && menu.focus > 0 { menu.focus -= 1; menu.dirty = true; }
    if nav.down && menu.focus + 1 < n { menu.focus += 1; menu.dirty = true; }
    let cur = f.get(menu.focus.min(n.saturating_sub(1))).copied();
    let dir = nav.right as i32 - nav.left as i32;
    if dir != 0 {
        match cur {
            Some(Item::Tiles) => { let i = MODES.iter().position(|m| *m == menu.tile).unwrap() as i32; menu.tile = MODES[(i + dir).clamp(0, 2) as usize]; menu.dirty = true; }
            Some(Item::Robot) => acts.0.push(Act::StepRobot(dir)),
            Some(Item::Opt(id)) => { cycle(id, &mut settings, dir); settings.save(); menu.dirty = true; }
            Some(Item::Btn { half: true, .. }) => {
                let j = menu.focus as i32 + dir;
                if j >= 0 && (j as usize) < n && matches!(f[j as usize], Item::Btn { half: true, .. }) { menu.focus = j as usize; menu.dirty = true; }
            }
            _ => {}
        }
    }
    if nav.ok {
        match cur {
            Some(Item::Tiles) => acts.0.push(Act::Tile(menu.tile)),
            Some(Item::Robot) => { menu.focus += 1; menu.dirty = true; }
            Some(Item::Opt(id)) => acts.0.push(Act::Step(id, 1)),
            Some(Item::Btn { act, .. }) => acts.0.push(act),
            _ => {}
        }
    }
    if nav.back && menu.page != Page::Home {
        let from = menu.page;
        open(&mut menu, Page::Home);
        if from == Page::Practice { menu.tile = Mode::Practice; menu.focus = 0; }
    }
}

fn open(menu: &mut Menu, page: Page) {
    menu.page = page;
    menu.focus = 0;
    if page == Page::Home { menu.focus = 0; }
    menu.dirty = true;
}

fn apply_acts(
    mut acts: ResMut<Acts>,
    mut menu: ResMut<Menu>,
    mut settings: ResMut<Settings>,
    mut controls: ResMut<ControlsOpen>,
    mut cmds: MessageWriter<GameCmd>,
    mut exit: MessageWriter<AppExit>,
) {
    let roster = Roster::get();
    for a in std::mem::take(&mut acts.0) {
        match a {
            Act::Open(p) => open(&mut menu, p),
            Act::Tile(m) => {
                menu.tile = m;
                if m.ready() { open(&mut menu, Page::Practice); } else { menu.dirty = true; }
            }
            Act::Start => {
                if roster.robot(&settings.robot).ported { cmds.write(GameCmd::Start); }
                else { menu.dirty = true; }
            }
            Act::Controls => controls.0 = true,
            Act::Quit => { exit.write(AppExit::Success); }
            Act::StepRobot(d) => {
                let n = roster.robots.len() as i32;
                let i = roster.robots.iter().position(|r| r.key == settings.robot).unwrap_or(0) as i32;
                settings.robot = roster.robots[(((i + d) % n + n) % n) as usize].key.clone();
                settings.save();
                menu.dirty = true;
            }
            Act::SetRobot(i) => { settings.robot = roster.robots[i].key.clone(); settings.save(); menu.dirty = true; }
            Act::Step(id, d) => { cycle(id, &mut settings, d); settings.save(); menu.dirty = true; }
            Act::Set(id, j) => {
                if let Some(v) = opt_values(id).get(j) { opt_set(id, &mut settings, &v.0); settings.save(); }
                menu.dirty = true;
            }
        }
    }
}

fn on_click(mut ev: On<PointerClick>, q: Query<&Click>, mut acts: ResMut<Acts>) {
    if let Ok(c) = q.get(ev.entity) {
        acts.0.push(c.0);
        ev.propagate(false);
    }
}

fn on_over(ev: On<PointerOver>, q: Query<&Focus>, mut menu: ResMut<Menu>) {
    if let Ok(f) = q.get(ev.entity) {
        if menu.focus != f.0 { menu.focus = f.0; menu.dirty = true; }
    }
}

/// js ui.js _desc: the line under the page about what's focused
fn describe(item: Option<Item>, menu: &Menu, s: &Settings) -> String {
    let r = Roster::get().robot(&s.robot);
    match item {
        Some(Item::Tiles) => format!("{} ←/→ choose · Enter open", menu.tile.desc()),
        Some(Item::Robot) => "Your robot. ←/→ or the arrows step through them; click any team to jump to it. ● marks robots simulated part by part.".into(),
        Some(Item::Opt(id)) => opt_desc(id, s),
        Some(Item::Btn { act: Act::Start, .. }) => if r.ported {
            format!("{} on the {} ALLIANCE, field to yourself.", r.short(), s.alliance.to_uppercase())
        } else {
            format!("{} can't play in this version yet: its mechanisms are still being ported. Pick 2910 to drive.", r.short())
        },
        Some(Item::Btn { act: Act::Quit, .. }) => "Close the game. A downloaded update installs as it closes.".into(),
        _ => String::new(),
    }
}

// ------------------------------------------------------------------ drawing
fn rebuild(
    mut commands: Commands,
    mut menu: ResMut<Menu>,
    settings: Res<Settings>,
    fonts: Res<Fonts>,
    roots: Query<Entity, With<MenuRoot>>,
    studio: Option<Res<crate::showcase::ShowcaseImage>>,
) {
    if !menu.dirty { return }
    menu.dirty = false;
    for e in &roots { commands.entity(e).despawn(); }
    let page = menu.page;
    let all = items(page);
    let n = all.iter().filter(|i| !matches!(i, Item::Section(_))).count();
    menu.focus = menu.focus.min(n.saturating_sub(1));
    let focus = menu.focus;
    let cur = focusables(page).get(focus).copied();
    let desc = describe(cur, &menu, &settings);
    let f = fonts.clone();

    // the overlay and its box (css .overlay, .menuBox)
    commands.spawn((
        MenuRoot,
        Node { width: Val::Percent(100.0), height: Val::Percent(100.0), position_type: PositionType::Absolute, align_items: AlignItems::Center, justify_content: JustifyContent::Center, ..default() },
        BackgroundColor(color::rgba(5, 6, 10, 0.72)),
        GlobalZIndex(10),
    )).with_children(|o| {
        o.spawn((
            Node {
                width: Val::Px(980.0), max_width: Val::Percent(94.0), max_height: Val::Percent(94.0),
                flex_direction: FlexDirection::Column, padding: UiRect::axes(Val::Px(26.0), Val::Px(22.0)),
                border: UiRect::all(Val::Px(1.0)), border_radius: radius(16.0), ..default()
            },
            BackgroundColor(color::BG), BorderColor::all(color::LINE),
        )).with_children(|b| {
            // title: the brand on the home page, else "‹ Menu  Page"
            if page == Page::Home {
                b.spawn(Node { flex_direction: FlexDirection::Row, align_items: AlignItems::Baseline, column_gap: Val::Px(16.0), margin: UiRect::bottom(Val::Px(14.0)), flex_wrap: FlexWrap::Wrap, ..default() })
                    .with_children(|t| {
                        t.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(8.0), ..default() }).with_children(|l| {
                            l.spawn(f.text("REBUILT", Face::Barlow(800), 44.0, color::TEXT));
                            l.spawn(f.text("2026", Face::Barlow(800), 44.0, color::FUEL));
                        });
                        t.spawn(f.text("FRC match simulator · practice, 1 v 1 or full 3 v 3 alliances · Xbox controller ready", Face::Inter(400), 14.0, color::MUTED));
                    });
            } else {
                b.spawn(Node { flex_direction: FlexDirection::Row, align_items: AlignItems::Center, column_gap: Val::Px(14.0), margin: UiRect::bottom(Val::Px(12.0)), ..default() })
                    .with_children(|t| {
                        t.spawn((f.text("‹ Menu", Face::Inter(700), 15.0, color::MUTED), Click(Act::Open(Page::Home)))).observe(on_click);
                        t.spawn(f.text(match page { Page::Practice => "Practice", Page::Settings => "Settings", Page::Home => "" }, Face::Barlow(800), 34.0, color::TEXT));
                    });
            }
            // the page (css #options: a two-column grid)
            let mut foot = Vec::new();
            b.spawn(Node {
                display: Display::Grid, grid_template_columns: vec![GridTrack::flex(1.0), GridTrack::flex(1.0)],
                column_gap: Val::Px(16.0), row_gap: Val::Px(6.0), overflow: Overflow::clip(), ..default()
            }).with_children(|g| {
                let mut fi = 0;
                for it in &all {
                    let idx = if matches!(it, Item::Section(_)) { usize::MAX } else { fi += 1; fi - 1 };
                    let focused = idx == focus;
                    match *it {
                        Item::Section(label) => section(g, &f, label),
                        Item::Tiles => tiles(g, &f, menu.tile, focused, idx),
                        Item::Robot => picker(g, &f, &settings, focused, idx, studio.as_ref().map(|s| s.0.clone())),
                        Item::Opt(id) => opt_row(g, &f, &settings, id, focused, idx),
                        Item::Btn { label, act, secondary, half, foot: true } => foot.push((label, act, secondary, half, focused, idx)),
                        Item::Btn { label, act, secondary, half, .. } => {
                            let dim = act == Act::Start && !Roster::get().robot(&settings.robot).ported;
                            button(g, &f, label, act, secondary, half, focused, idx, dim);
                        }
                    }
                }
            });
            // the footer: what's focused, then START / BACK (css #menuActions)
            b.spawn(Node { flex_direction: FlexDirection::Column, border: UiRect::top(Val::Px(1.0)), padding: UiRect::top(Val::Px(10.0)), margin: UiRect::top(Val::Px(8.0)), ..default() })
                .insert(BorderColor::all(color::LINE))
                .with_children(|a| {
                    a.spawn((Node { min_height: Val::Px(36.0), padding: UiRect::bottom(Val::Px(8.0)), ..default() },)).with_children(|d| {
                        d.spawn(f.text(desc, Face::Inter(400), 13.0, color::TEXT));
                    });
                    if !foot.is_empty() {
                        a.spawn(Node { display: Display::Grid, grid_template_columns: vec![GridTrack::flex(1.0), GridTrack::px(200.0)], column_gap: Val::Px(10.0), ..default() })
                            .with_children(|r| {
                                for (label, act, secondary, _, focused, idx) in foot {
                                    let dim = act == Act::Start && !Roster::get().robot(&settings.robot).ported;
                                    button(r, &f, label, act, secondary, true, focused, idx, dim);
                                }
                            });
                    }
                    a.spawn((f.text("", Face::Inter(400), 13.0, color::MUTED), Node { margin: UiRect::top(Val::Px(10.0)), ..default() }, FootStatus));
                });
        });
    });
}

fn foot_status(mut q: Query<&mut Text, With<FootStatus>>, pads: Query<&Name, With<Gamepad>>, upd: Res<UpdateStatus>) {
    let pad = pads.iter().next().map(|n| format!("Controller connected: {n}")).unwrap_or_else(|| "Press any button on your Xbox controller to connect it (keyboard works too)".into());
    let s = format!("{pad} · {}", upd.text());
    for mut t in &mut q { if t.0 != s { t.0 = s.clone(); } }
}

fn section(g: &mut ChildSpawnerCommands, f: &Fonts, label: &str) {
    g.spawn((
        Node { grid_column: GridPlacement::span(2), border: UiRect::bottom(Val::Px(1.0)), padding: UiRect::bottom(Val::Px(3.0)), margin: UiRect::top(Val::Px(10.0)), ..default() },
        BorderColor::all(color::LINE),
    )).with_children(|s| { s.spawn(f.text(label.to_uppercase(), Face::Barlow(700), 15.0, color::MUTED)); });
}

/// css .tiles / .tile
fn tiles(g: &mut ChildSpawnerCommands, f: &Fonts, sel: Mode, focused: bool, idx: usize) {
    g.spawn((Node { grid_column: GridPlacement::span(2), display: Display::Grid, grid_template_columns: RepeatedGridTrack::flex(3, 1.0), column_gap: Val::Px(14.0), padding: UiRect::all(Val::Px(3.0)), margin: UiRect::bottom(Val::Px(8.0)), ..default() }, Focus(idx)))
        .observe(on_over)
        .with_children(|t| {
            for m in MODES {
                let on = m == sel;
                let border = if on && focused { color::FUEL } else if on { color::over(244, 210, 58, 0.6) } else { color::LINE };
                let bg = if on { color::over(244, 210, 58, 0.07) } else { color::tint(0.03) };
                let mut e = t.spawn((
                    Node { flex_direction: FlexDirection::Column, align_items: AlignItems::Center, padding: UiRect::new(Val::Px(16.0), Val::Px(16.0), Val::Px(22.0), Val::Px(18.0)), border: UiRect::all(Val::Px(2.0)), border_radius: radius(14.0), ..default() },
                    BackgroundColor(bg), BorderColor::all(border), Click(Act::Tile(m)),
                ));
                if on && focused { e.insert(Outline::new(Val::Px(3.0), Val::Px(1.0), color::over(47, 111, 240, 0.55))); }
                e.observe(on_click).with_children(|c| {
                    icon(c, m);
                    c.spawn(f.text(m.name(), Face::Barlow(800), 34.0, if m.ready() { color::TEXT } else { color::MUTED }));
                    c.spawn(f.text(if m.ready() { m.line() } else { "Not in this version yet" }, Face::Inter(400), 13.0, color::MUTED));
                });
            }
        });
}

/// The tiles' icons (◎ ⚔ ⬢ in the browser), drawn: a ring, two crossed bars, a hexagon-ish disc
fn icon(c: &mut ChildSpawnerCommands, m: Mode) {
    c.spawn(Node { width: Val::Px(34.0), height: Val::Px(34.0), margin: UiRect::bottom(Val::Px(8.0)), align_items: AlignItems::Center, justify_content: JustifyContent::Center, ..default() })
        .with_children(|i| match m {
            Mode::Practice => {
                i.spawn((Node { width: Val::Px(28.0), height: Val::Px(28.0), border: UiRect::all(Val::Px(3.0)), border_radius: radius(14.0), align_items: AlignItems::Center, justify_content: JustifyContent::Center, ..default() }, BorderColor::all(color::FUEL)))
                    .with_children(|r| { r.spawn((Node { width: Val::Px(10.0), height: Val::Px(10.0), border: UiRect::all(Val::Px(3.0)), border_radius: radius(5.0), ..default() }, BorderColor::all(color::FUEL))); });
            }
            Mode::OneVOne => {
                for a in [0.785f32, -0.785] {
                    i.spawn((Node { position_type: PositionType::Absolute, width: Val::Px(30.0), height: Val::Px(4.0), border_radius: radius(2.0), ..default() }, BackgroundColor(color::FUEL), UiTransform::from_rotation(Rot2::radians(a))));
                }
            }
            Mode::ThreeVThree => {
                for a in [0.0f32, 1.047, 2.094] {
                    i.spawn((Node { position_type: PositionType::Absolute, width: Val::Px(17.0), height: Val::Px(29.0), ..default() }, BackgroundColor(color::FUEL), UiTransform::from_rotation(Rot2::radians(a))));
                }
            }
        });
}

/// css .btn
#[allow(clippy::too_many_arguments)]
fn button(g: &mut ChildSpawnerCommands, f: &Fonts, label: &str, act: Act, secondary: bool, half: bool, focused: bool, idx: usize, dim: bool) {
    // (a half button takes one of the page's two columns; the footer's grid has a column each)
    let span = if half { 1 } else { 2 };
    let (bg, fg, size) = if secondary { (color::tint(0.08), color::TEXT, 18.0) } else { (color::FUEL, color::DARK, 24.0) };
    let bg = if dim { bg.with_alpha(0.35) } else { bg };
    g.spawn((
        Node { grid_column: GridPlacement::span(span), justify_content: JustifyContent::Center, align_items: AlignItems::Center, padding: UiRect::all(Val::Px(if secondary { 12.0 } else { 10.0 })), border: UiRect::all(Val::Px(3.0)), border_radius: radius(10.0), ..default() },
        BackgroundColor(bg), BorderColor::all(if focused { color::tint(0.9) } else { Color::NONE }), Focus(idx), Click(act),
    )).observe(on_over).observe(on_click).with_children(|b| { b.spawn(f.text(label, Face::Barlow(800), size, fg)); });
}

/// css .opt with a stepper or segments
fn opt_row(g: &mut ChildSpawnerCommands, f: &Fonts, s: &Settings, id: OptId, focused: bool, idx: usize) {
    let vals = opt_values(id);
    let cur = opt_get(id, s);
    let ci = vals.iter().position(|v| v.0 == cur).unwrap_or(0);
    g.spawn((
        Node { flex_direction: FlexDirection::Row, align_items: AlignItems::Center, justify_content: JustifyContent::SpaceBetween, column_gap: Val::Px(10.0), min_height: Val::Px(46.0), padding: UiRect::new(Val::Px(12.0), Val::Px(8.0), Val::Px(6.0), Val::Px(6.0)), border: UiRect::all(Val::Px(1.0)), border_radius: radius(8.0), ..default() },
        BackgroundColor(if focused { color::over(47, 111, 240, 0.12) } else { color::tint(0.03) }),
        BorderColor::all(if focused { color::BLUE } else { Color::NONE }), Focus(idx),
    )).observe(on_over).with_children(|o| {
        o.spawn(f.text(opt_label(id), Face::Inter(400), 13.0, color::MUTED));
        if segmented(id) {
            o.spawn((Node { flex_direction: FlexDirection::Row, padding: UiRect::all(Val::Px(2.0)), column_gap: Val::Px(2.0), border: UiRect::all(Val::Px(1.0)), border_radius: radius(8.0), ..default() }, BackgroundColor(color::over(0, 0, 0, 0.28)), BorderColor::all(color::LINE)))
                .with_children(|sg| {
                    for (j, (v, label)) in vals.iter().enumerate() {
                        let on = j == ci;
                        let (bg, fg) = match (on, v.as_str()) {
                            (true, "blue") => (color::BLUE, Color::WHITE),
                            (true, "red") => (color::RED, Color::WHITE),
                            (true, _) => (color::tint(0.9), color::DARK),
                            _ => (Color::NONE, color::MUTED),
                        };
                        sg.spawn((Node { padding: UiRect::axes(Val::Px(12.0), Val::Px(6.0)), border_radius: radius(6.0), ..default() }, BackgroundColor(bg), Click(Act::Set(id, j))))
                            .observe(on_click)
                            .with_children(|t| { t.spawn(f.text(label.clone(), Face::Inter(600), 13.5, fg)); });
                    }
                });
        } else {
            stepper(o, f, &vals[ci].1, Act::Step(id, -1), Act::Step(id, 1), Some(Act::Step(id, 1)));
        }
    });
}

/// css .stepper: ‹ value ›
fn stepper(o: &mut ChildSpawnerCommands, f: &Fonts, value: &str, prev: Act, next: Act, on_value: Option<Act>) {
    o.spawn(Node { display: Display::Grid, grid_template_columns: vec![GridTrack::px(32.0), GridTrack::flex(1.0), GridTrack::px(32.0)], align_items: AlignItems::Center, column_gap: Val::Px(4.0), width: Val::Px(270.0), max_width: Val::Percent(64.0), ..default() })
        .with_children(|st| {
            let arrow = |st: &mut ChildSpawnerCommands, glyph: &str, act: Act| {
                st.spawn((Node { height: Val::Px(32.0), justify_content: JustifyContent::Center, align_items: AlignItems::Center, border: UiRect::all(Val::Px(1.0)), border_radius: radius(7.0), ..default() }, BackgroundColor(color::tint(0.08)), BorderColor::all(color::LINE), Click(act)))
                    .observe(on_click).with_children(|a| { a.spawn(f.text(glyph, Face::Inter(600), 20.0, color::TEXT)); });
            };
            arrow(st, "‹", prev);
            let mut v = st.spawn((Node { justify_content: JustifyContent::Center, padding: UiRect::all(Val::Px(6.0)), overflow: Overflow::clip(), ..default() },));
            if let Some(a) = on_value { v.insert(Click(a)).observe(on_click); }
            v.with_children(|t| { t.spawn((f.text(value, Face::Inter(600), 14.0, color::TEXT), TextLayout::no_wrap())); });
            arrow(st, "›", next);
        });
}

/// js ui.js _picker: the robot's card between big arrows, and every robot in a strip under it
fn picker(g: &mut ChildSpawnerCommands, f: &Fonts, s: &Settings, focused: bool, idx: usize, studio: Option<Handle<Image>>) {
    let roster = Roster::get();
    let r = roster.robot(&s.robot);
    let n = roster.robots.iter().position(|x| x.key == r.key).unwrap_or(0);
    g.spawn((
        Node { grid_column: GridPlacement::span(2), flex_direction: FlexDirection::Column, padding: UiRect::all(Val::Px(8.0)), border: UiRect::all(Val::Px(1.0)), border_radius: radius(14.0), ..default() },
        BackgroundColor(color::tint(0.02)), BorderColor::all(if focused { color::BLUE } else { color::LINE }), Focus(idx),
    )).observe(on_over).with_children(|p| {
        p.spawn(Node { display: Display::Grid, grid_template_columns: vec![GridTrack::px(48.0), GridTrack::flex(1.0), GridTrack::px(48.0)], column_gap: Val::Px(8.0), ..default() })
            .with_children(|m| {
                let arrow = |m: &mut ChildSpawnerCommands, glyph: &str, act: Act| {
                    m.spawn((Node { justify_content: JustifyContent::Center, align_items: AlignItems::Center, border: UiRect::all(Val::Px(1.0)), border_radius: radius(10.0), ..default() }, BackgroundColor(color::tint(0.06)), BorderColor::all(color::LINE), Click(act)))
                        .observe(on_click).with_children(|a| { a.spawn(f.text(glyph, Face::Inter(300), 40.0, color::TEXT)); });
                };
                arrow(m, "‹", Act::StepRobot(-1));
                // the card (css .pkCard)
                m.spawn((Node { flex_direction: FlexDirection::Column, min_height: Val::Px(196.0), padding: UiRect::axes(Val::Px(16.0), Val::Px(12.0)), border: UiRect::all(Val::Px(2.0)), border_radius: radius(12.0), ..default() }, BackgroundColor(color::over(244, 210, 58, 0.06)), BorderColor::all(color::over(244, 210, 58, 0.55))))
                    .with_children(|c| {
                        c.spawn(Node { flex_direction: FlexDirection::Row, align_items: AlignItems::Center, column_gap: Val::Px(14.0), ..default() }).with_children(|h| {
                            h.spawn(f.text(r.team.to_string(), Face::Barlow(800), 52.0, color::TEXT));
                            h.spawn(Node { flex_direction: FlexDirection::Column, flex_grow: 1.0, row_gap: Val::Px(4.0), ..default() }).with_children(|t| {
                                t.spawn(f.text(r.long_name(), Face::Inter(600), 16.0, color::TEXT));
                                t.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(6.0), ..default() }).with_children(|tags| {
                                    tag(tags, f, &r.archetype.to_uppercase(), color::tint(0.1), color::TEXT);
                                    tag(tags, f, "FULL PHYSICS", color::over(55, 214, 122, 0.18), color::over(0x9f, 0xf0, 0xc0, 1.0));
                                    if !r.ported { tag(tags, f, "NOT IN THIS VERSION YET", color::over(255, 176, 32, 0.2), color::WARN); }
                                });
                            });
                            h.spawn(f.text(format!("{} / {}", n + 1, roster.robots.len()), Face::Inter(600), 12.5, color::MUTED));
                        });
                        // the robot on its turntable beside what it says of itself (showcase.rs)
                        c.spawn(Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(14.0), align_items: AlignItems::FlexStart, ..default() }).with_children(|row| {
                            row.spawn(Node { flex_direction: FlexDirection::Column, flex_grow: 1.0, flex_basis: Val::Px(0.0), ..default() }).with_children(|l| {
                                l.spawn((Node { margin: UiRect::new(Val::Px(0.0), Val::Px(0.0), Val::Px(8.0), Val::Px(10.0)), min_height: Val::Px(38.0), ..default() },)).with_children(|d| {
                                    d.spawn(f.text(r.blurb.clone(), Face::Inter(400), 13.0, color::MUTED));
                                });
                                l.spawn(Node { flex_direction: FlexDirection::Row, flex_wrap: FlexWrap::Wrap, column_gap: Val::Px(18.0), row_gap: Val::Px(4.0), ..default() }).with_children(|st| {
                                    for (k, v) in &r.stats {
                                        if v.is_empty() { continue }
                                        st.spawn((Node { width: Val::Px(220.0), flex_direction: FlexDirection::Row, justify_content: JustifyContent::SpaceBetween, column_gap: Val::Px(8.0), border: UiRect::bottom(Val::Px(1.0)), padding: UiRect::vertical(Val::Px(1.0)), ..default() }, BorderColor::all(color::tint(0.05))))
                                            .with_children(|row| {
                                                row.spawn(f.text(k.clone(), Face::Inter(400), 12.5, color::MUTED));
                                                row.spawn((f.text(v.clone(), Face::Inter(700), 12.5, color::TEXT), TextLayout::no_wrap()));
                                            });
                                    }
                                });
                            });
                            if let Some(img) = studio.clone() {
                                row.spawn((ImageNode::new(img), Node { width: Val::Px(crate::showcase::W as f32 / 2.0), height: Val::Px(crate::showcase::H as f32 / 2.0), flex_shrink: 0.0, ..default() }));
                            }
                        });
                    });
                arrow(m, "›", Act::StepRobot(1));
            });
        // every robot (css .pkStrip / .chip), scrolled so the chosen one is in the middle
        p.spawn((Node { flex_direction: FlexDirection::Row, column_gap: Val::Px(6.0), margin: UiRect::top(Val::Px(8.0)), padding: UiRect::new(Val::Px(2.0), Val::Px(2.0), Val::Px(2.0), Val::Px(6.0)), overflow: Overflow::scroll_x(), ..default() }, ScrollPosition::default(), Strip(n)))
            .with_children(|strip| {
                for (i, q) in roster.robots.iter().enumerate() {
                    let sel = i == n;
                    let a = if q.ported { 1.0 } else { 0.55 };
                    strip.spawn((
                        Node { flex_shrink: 0.0, width: Val::Px(92.0), flex_direction: FlexDirection::Column, align_items: AlignItems::Center, padding: UiRect::axes(Val::Px(8.0), Val::Px(6.0)), border: UiRect::all(Val::Px(2.0)), border_radius: radius(9.0), ..default() },
                        BackgroundColor(if sel { color::over(244, 210, 58, 0.14) } else { color::tint(0.03) }),
                        BorderColor::all(if sel { color::FUEL } else { color::LINE }), Click(Act::SetRobot(i)),
                    )).observe(on_click).with_children(|c| {
                        c.spawn(f.text(q.team.to_string(), Face::Barlow(800), 22.0, color::TEXT.with_alpha(a)));
                        c.spawn((f.text(q.archetype.clone(), Face::Inter(400), 10.5, if sel { color::TEXT } else { color::MUTED.with_alpha(a) }), TextLayout::no_wrap()));
                        // (the green dot: simulated part by part)
                        c.spawn((Node { position_type: PositionType::Absolute, top: Val::Px(5.0), right: Val::Px(5.0), width: Val::Px(6.0), height: Val::Px(6.0), border_radius: radius(3.0), ..default() }, BackgroundColor(color::OK.with_alpha(a))));
                    });
                }
            });
    });
}

fn tag(p: &mut ChildSpawnerCommands, f: &Fonts, label: &str, bg: Color, fg: Color) {
    p.spawn((Node { padding: UiRect::axes(Val::Px(8.0), Val::Px(2.0)), border_radius: radius(99.0), ..default() }, BackgroundColor(bg)))
        .with_children(|t| { t.spawn(f.text(label, Face::Inter(700), 11.0, fg)); });
}

/// The chosen chip in the middle of the strip
fn center_strip(mut q: Query<(&Strip, &ComputedNode, &mut ScrollPosition)>) {
    for (s, node, mut pos) in &mut q {
        let w = node.size().x * node.inverse_scale_factor();
        if w <= 0.0 { continue }
        let x = (s.0 as f32 * 98.0 - (w - 92.0) / 2.0).max(0.0);
        if (pos.0.x - x).abs() > 0.5 { pos.0.x = x; }
    }
}
