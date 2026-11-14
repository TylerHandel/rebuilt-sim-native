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

## What it needs, in order

1. **1v1 and 3v3 in the native game:** the rules between robots, the AI drivers (navigation,
   their strategies and skill levels), and the teams' autos, each ported identical to the
   browser game. *(Done; the trained network driver is still to come.)*
2. **Robot ratings:** the round robin, run headless, saved as a table.
3. **The event engine:** schedules, ranking points, decided matches, alliance selection, the
   double-elimination bracket, district points, save and resume. Pure logic, tested headless.
4. **The screens:** the season map, the event screen (pits), pre- and post-match, the coach's
   lines, awards, the recap; the arena's screens showing the event.
5. **The season:** advancement, the rival, the season score.
