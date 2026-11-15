# Road to Einstein

The campaign: one season of FRC REBUILT, from your first district event to the Einstein field at
the World Championship, driving one robot the whole way. It's built on the same match the other
modes play (1v1 and 3v3 against the AI drivers), with an event around each match: a schedule,
rankings, alliance selection, a playoff bracket, and a season that carries from one event to the
next.

## The season

```
Week 1 District ─┐
                 ├─► District Championship ─► Championship Division ─► Einstein
Week 3 District ─┘   (top district points)     (top division seeds)      (division winners)
```

- **Pick your robot** at the start: any of the 17. Each has a rating (see Difficulty); a weaker
  robot makes the season harder and is worth more season score. You drive it all season.
- **District events** (two): 8 qualification matches, alliance selection, a double-elimination
  playoff of 8 alliances, finals best of three. District points (as FRC awards them: qualification
  record, alliance selection, playoff advancement, awards) decide who goes on.
- **District Championship:** a bigger, stronger field; 10 qualification matches, the same playoff.
- **Championship Division:** the strongest robots and drivers; the division winner goes to Einstein.
- **Einstein:** a round robin of the division winners, then the finals.

Missing the cut ends the season there (with a "next season" option to try again); a quick season
(one district event straight to Einstein) is there for a shorter session.

## A match in an event

- **3v3, as FRC plays it.** Qualification alliances are drawn by the schedule (your partners and
  opponents change each match); playoff alliances are picked.
- **Your partners and opponents are AI drivers** (the same ones 1v1 and 3v3 use), each with its own
  robot, skill level and strategy (scorer, defender, hybrid). Partners follow a plan you can set
  before the match: who plays defense, who feeds, which side each starts on.
- **Only your matches are played.** Every other match at the event is decided from the robots'
  ratings and drivers' skills with some luck, so the rankings move as a real event's would.
- **Lower-end PCs:** an event can be played as 1v1 instead (your robot against each opponent
  alliance's best robot), with the same schedule and bracket.

## Between matches: the pits

The event screen between matches:

- **Next match:** your partners and opponents, each robot's card (rating, cycle speed, capacity,
  climb, whether it plays defense) and a strategy picker for your alliance.
- **Schedule, rankings, bracket:** live, updated after each match.
- **Scouting:** what every robot did in its matches here (from the decided matches too), which
  is what alliance selection runs on.
- **Save and resume:** the season saves after every match.

## Alliance selection

If you seed in the top 8 you pick partners, in FRC's serpentine order, from the scouting data;
the AI captains pick by rating and need (a defender for a scoring alliance, and so on). If you
don't, you may be picked, or decline to be. The arena's screens show the picks.

## The story

You're the new driver for a team, and the season is told around the matches, never instead of
them:

- **Your coach** talks before and after each match: a short plan, what went wrong, what the
  scouts saw. Their lines come from what happened (a pin foul, a missed climb, a big auto).
- **A rival:** one AI team (its robot and driver) that turns up at your events, gets better as the
  season goes on, and that you meet in the playoffs if you both get there.
- **Awards** at each event, from what you did: high score, best auto, the defense award, and
  recognition for clean play (few fouls).
- **The season recap** at the end: your record, the matches that mattered, your season score.

## Difficulty

Three things decide how hard a match is, combined into one expected strength per alliance:

1. **The robots:** each robot's rating, from a headless round robin (every robot against every
   robot, several seeds, both sides).
2. **The drivers:** the AI skill levels (Rookie, Regional, Champs, Trained), harder as the season
   goes: districts mostly Rookie and Regional, the Championship Champs and Trained.
3. **The setting:** Casual, Standard or Hall of Fame shifts the target win chances (about 70% at a
   district, 50% at the District Championship, 35% on Einstein on Standard).

## Season score

Points for each event finish and award, times a robot multiplier (the weaker your robot's rating,
the higher), times the difficulty's. It's what the season recap ranks you by.

## Where it stands

1. **1v1 and 3v3 in the native game:** the rules between robots, the AI drivers (navigation,
   their strategies and skill levels), and the teams' autos, each ported identical to the browser
   game. *Done; the trained network driver is still to come.*
2. **Robot ratings:** `crates/sim/examples/ratings.rs` plays every design headless with the Champs
   AI: 3 v 3 matches of mixed alliances (OPR, with AUTO, FUEL and TOWER apart, and the spread of
   alliances' scores around their OPRs) and a 1 v 1 round robin from both sides (Bradley-Terry
   strengths on the Elo scale). Written to `assets/ratings.json`. *Done.*
3. **The event engine** (`crates/campaign`, pure logic, tested headless): an even qualification
   schedule with surrogates, decided matches (the teams' ratings and drivers' skills with luck),
   ranking points by the 2026 rules, the rankings and their tiebreakers, serpentine alliance
   selection (declines, a lower captain picked), the eight-alliance double elimination, Einstein's
   round robin and finals, district points, awards, save and resume. *Done.*
4. **The screens:** the season page on the menu, the pits (your next match, both alliances, the
   coach, the rankings, your results), alliance selection, the playoffs, the event's end, the
   recap. The pits have tabs: Next (with your partners' roles for the match, each changeable:
   scorer, defense or hybrid), Schedule (your matches in full, then every match), Rankings (all of
   them), Playoffs (the alliances and every bout) and Scouting (each team's robot, its rating, its
   drivers, how it plays and its average here). The arena's screens show the event, the match
   (Q12, M5, F2) and the teams by name. *Done.*
5. **The season:** a district of 48 fictional teams fielding the robot designs over four district
   events (you at two, the rival at both), the District Championship (top 32), the Championship
   (the district's top 10 and the DCMP winners, in a division of 36), Einstein (four division
   winners). A short season is one district event, then straight to the Championship for its
   winners and its five best in district points. When a season ends, the next one starts with
   the same team, robot and difficulty in a new district, the seasons before it in its recap.
   *Done.*
6. **1 v 1 matches (for slow PCs):** your matches with just you and the other alliance's best
   robot on the field; your partners' points and the rest of theirs are decided from their
   ratings (the luck scaled to how many of the alliance they are) and added to the two robots'.
   *Done.*

### Difficulty, as calibrated

`crates/campaign/examples/calibrate.rs` plays hundreds of seasons on the ratings, your matches
decided as a Champs-level driver's would be. With a mid-pack robot (1690) on Standard you win
about 65% of qualification matches at districts, 52% at the District Championship and 39% in a
Championship division; Casual about 78 / 65 / 46, Hall of Fame 54 / 44 / 30. How far you get
depends on you, of course: these are the AI's numbers in your seat.

### Scaled to the simulator

This simulator's alliances score several times the FUEL a real REBUILT alliance does (a
Champs-level 3 v 3 alliance: about 790), so the ENERGIZED and SUPERCHARGED thresholds are scaled
to it (district 450 / 750, District Championship and Championship 650 / 900). TOWER points are the
real game's, and so is TRAVERSAL's threshold (50, then 90).
