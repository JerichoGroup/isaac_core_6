# Style profile — questions for Ofer

Answer inline, or reply "all recommended except 3, 7". Each question has a recommendation and the
cost of applying it. Nothing in Phase 1 or 2 starts until this is settled, because both phases touch
almost every file and I do not want to do that twice.

Measured baseline: 15,197 lines of source, 538 multi-line docstrings, 395 one-line docstrings,
364 docstrings with `Args:`/`Returns:`/`Raises:` sections, 144 modules with docstrings.

---

## Part A — Code style

### A1. Docstring summary position ← the one that matters most

We currently **ignore `D212`**, which means we enforce the summary on the **second** line:

```python
"""
Return the geodetic pose as a tuple.

Body paragraph.
"""
```

PEP 257 and the Google style guide both put the summary on the **first** line:

```python
"""Return the geodetic pose as a tuple.

Body paragraph.
"""
```

You said docstrings should follow the industry standard. Our current choice is not it, and I should
have flagged that when it was set.

- **(a) Switch to first-line summaries (PEP 257).** 538 docstrings affected, but `ruff --fix`
  autofixes `D212` mechanically and the test suite verifies nothing broke.
- (b) Keep second-line summaries.

**Recommendation: (a).** It is the standard, it is what any new contributor expects, and the change
is automated rather than hand-edited.

### A2. One-line vs multi-line docstrings

- (a) One line wherever the function is self-explanatory; expand only when there is something a
  reader could get wrong.
- (b) Always full `Args:`/`Returns:` sections, as now (364 currently have them).

**Recommendation: (a)** for private helpers and obvious functions, **(b)** for anything a user calls
(devkit, CLI, config models). Public API keeps full sections; internals get one line. This cuts noise
where it does not earn its place and keeps it where users actually read.

### A3. Module docstrings

144 modules have one. Keep requiring them?

- (a) Required, one line, saying what the module is for.
- (b) Required, and allowed to explain design where relevant.

**Recommendation: (a).** Design rationale belongs next to the code it explains, not in a module
preamble a reader skims past.

### A4. Comment policy

- (a) Comments explain **why**, one or two lines, no history, no incident reports.
- (b) Also allow short "how" comments for dense algorithms.

**Recommendation: (a) plus (b) limited to genuinely dense maths** (geodesy, quaternion composition).
Confirmed already: **no `# ===== Section ===== #` banner headers** anywhere.

### A5. What happens to the hard-won reasoning

Some comments exist to stop a future reader from *undoing* a decision that cost real time — the
gimbal frame composition, the 60 warm-up frames, the parallel-arrays message design.

- (a) Keep as a one-line "why", with detail in the development log.
- (b) Keep the full explanation in the code.

**Recommendation: (a).** The constraint stays visible; the story moves to the log.

### A6. Logging convention

2023 used `carb.log_info("SIM | GPTLP | ...")` in OGN nodes. We use the same prefix in nodes and the
standard `logging` module elsewhere.

- (a) Keep as is.
- (b) Unify on one format everywhere.

**Recommendation: (a).** OGN nodes log through carb into the Isaac console where the prefix is what
lets you find our lines; host-side code uses `logging` because that is what Python tooling expects.

### A7. Line length

Currently 120.

- (a) Keep 120.
- (b) Drop to 100 or 88 (Black default).

**Recommendation: (a).** Changing it reflows the entire codebase for no functional gain.

### A8. Naming conventions to keep or drop

Currently: `_deg`/`_r` unit suffixes on every angle, `_m` on distances, `Lla`/`Rpy` short types.

**Recommendation: keep all.** The unit suffixes exist because the wire is radians and the GUI is
degrees, which caused real confusion in 2023.

---

## Part B — README and documentation

### B1. Emoji in headings

2023 used them (`# Isaac Core 🌎📸`, `## System Requirements🖥️`).

- (a) None.
- (b) Top-level title only.
- (c) One per section heading, as 2023 did.

**Recommendation: (b).** Your call entirely — this is taste, and it is your team's repo.

### B2. Table of contents

2023 had one. Ours does not.

**Recommendation: yes**, since the README will stay long enough to need it.

### B3. Voice

- (a) Second person: "You launch the simulator with…"
- (b) Imperative: "Launch the simulator with…"

**Recommendation: (b)** for instructions, (a) when explaining a consequence. This is what the 2023
README does and it reads well.

### B4. Depth in the README vs linked docs

Currently 746 lines including a 120-line troubleshooting section and a full architecture section.

- (a) README covers install, run, every feature, topics, conventions, config, short troubleshooting.
  Architecture and deep explanation move to `docs/`.
- (b) Keep everything in one file.

**Recommendation: (a).** A user wants to find their feature fast; the architecture section is for
whoever extends the repo, which is a different reader on a different day.

### B5. Examples: CLI or Python first

- (a) CLI first (`isaac-core run`, `ros2 topic echo`), Python second.
- (b) Python devkit first.

**Recommendation: (a).** The fastest path to a moving picture involves no Python at all, and that
should be the first thing a new user succeeds at.

### B6. Feature section template

Proposed, uniform for all features:

```
### Feature name
One sentence: what it does.
How to enable it (config snippet).
How to use it (command or snippet).
How to confirm it works (command + expected output).
```

**Recommendation: adopt.** Predictable structure means users learn to scan it.

### B7. Screenshots — I need you here

Candidates, in priority order:

1. The viewport over Nablus terrain with the camera tracking a pose. **The most valuable image
   in the README**: it is the thing the repo does.
2. Gimbal angle reference, as 2023 had. Yours was genuinely good.
3. `rqt` showing the image topic live.
4. Bounding boxes drawn over the terrain.
5. The pose sender GUI.
6. Distance sensor reading in the console alongside the view.

Tell me which you want to shoot and I will write the exact caption and placement, and specify the
pose and config for each so they look consistent.

### B8. Migration-from-2023 content

Scattered through the README now (23 references to "2023" in source, several in the README).

- (a) One `docs/migrating_from_2023.md`, linked once.
- (b) Keep notes inline where each difference appears.

**Recommendation: (a).** New users are the majority audience and do not care; your team reads it
once.

### B9. Troubleshooting section

Currently ~120 lines and genuinely useful (segfaults, stale `$ISAACSIM_PATH`, missing terrain,
Cesium cache growth).

- (a) Keep the entries, compress each to symptom → cause → fix.
- (b) Move wholesale to `docs/troubleshooting.md` with the top five inline.

**Recommendation: (a)**, staying in the README. These are the failures people actually hit, and a
user in trouble should not need a second file.

### B10. Development log

`docs/development-log.md` is 3,372 lines at the repo root, written for us, and it is the first thing a newcomer opens
after the README.

- (a) Move to `docs/development-log.md`, keep appending.
- (b) Leave at root.
- (c) Trim to decisions only, drop the narrative.

**Recommendation: (a).** It has real value as the record of *why*, and none of it is user-facing.
Moving it costs nothing and (c) would destroy the part that is actually useful later.

---

## Part C — Two things I want your explicit call on

### C1. Config file as documentation

`config/default.toml` is heavily commented and doubles as the config reference. Keep it that way, or
strip it and document config in the README?

**Recommendation: keep it.** It is the one place users will definitely look, and inline comments
cannot drift out of sync with the values the way a separate table can.

### C2. Test naming

Tests currently read `test_gimbal_pitch_moves_camera_look_direction_down` — long and behavioural.

**Recommendation: keep.** When one fails, the name alone usually tells you what broke.
