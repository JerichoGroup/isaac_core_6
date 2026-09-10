# Version 2 finalization plan

Every feature works. This plan covers what stands between "works" and "a stranger can pick this up
and use it": documentation aimed at users instead of at us, a codebase with no leftovers, and a
verification pass over **every** capability the repo claims.

Ordered so that each phase produces something checkable, and so the riskiest edits happen while the
test suite is at its strongest.

Current baseline: **1560 tests**, all pre-commit hooks green, 23 import-linter contracts kept, zero
dead config keys.

---

## Phase 0 — Freeze the claim surface (half a day)

Everything else is measured against this, so it comes first.

Build `docs/feature_matrix.md`: one row per capability the repo claims, each with
**how it is verified**, **where the evidence lives**, and **who checks it**.

The rows come from three sources, so nothing is missed:

1. Every capability sentence in `README.md`.
2. Every `Method` enum member and every `SimSession` public method.
3. Every published ROS topic, every config section, and every shipped layer.

Each row lands in exactly one bucket:

- **A — automated**: a test asserts the behaviour with no Isaac Sim. Highest confidence, cheapest.
- **B — live-scripted**: needs a running simulator; verified by a script that asserts numbers.
- **C — eyes-on**: needs a human to look at a picture.

**Exit:** every claim has a row and a bucket. Any claim that cannot be assigned is either untrue or
untestable, and both are findings.

The whole matrix is **re-run in Phase 5**, after the cleanup. A matrix verified only before the
cleanup certifies code that no longer exists.

---

## Phase 0.5 — Adversarial review by independent agents (1 day)

A squad of sub-agents reviews the repo with fresh eyes and a mandate to be harsh: what is wrong, what
is missing, what tests we are not writing, what a stranger would trip over. I have the final say on
every finding, because a reviewer with no history here will confidently report things that are
deliberate — but a reviewer with no history is also the only one who can see what we have stopped
noticing.

### Why here, and not later

Findings split into two kinds, and the split determines the placement:

- **Substantive** — missing tests, wrong logic, weak assertions, API design problems, dead surface,
  robustness gaps. These must arrive **before** Phase 2 (cleanup) and Phase 3 (verification) to be
  actionable at all. A missing-test finding delivered after Phase 3 means doing Phase 3 twice.
- **Cosmetic** — comments, docstrings, naming, document structure. These would be *invalidated* by
  Phases 1 and 2, which rewrite exactly that material.

So the review runs immediately after the matrix, and is **explicitly scoped to substance**. Reviewers
are told to ignore comment and docstring style, because a dedicated phase already owns it and
critiquing it now wastes their attention and mine.

Running it before Phase 1 also means the README rewrite can answer the confusions a fresh reader
actually reports, rather than the ones I imagine they will have.

### Review tracks, run in parallel

1. **Architecture and public API** — layering, abstractions, whether the devkit is the API a user
   would want, what is over- or under-engineered.
2. **Test suite critic** — the important one. What is *not* tested, which assertions are weak enough
   to pass while broken, which tests would survive deleting the code they cover.
3. **Correctness of the pure kernel** — geodesy, rotations, packet codec, trajectories. Adversarial
   about edge cases: poles, antimeridian, gimbal lock, malformed input.
4. **Isaac-facing surface** — OGN nodes, USD layers, manifests, composition order, what breaks when
   Isaac's API shifts under us.
5. **Robustness and failure modes** — resource leaks, partial failures, concurrency, what happens
   when a dependency is missing or a socket dies mid-call.
6. **Fresh-user simulation** — follow the docs cold with no context and report every point of
   friction, wrong instruction and unstated assumption.
7. **Config and schema** — the whole config surface, defaults, validation, and whether the error
   messages tell a user what to fix.

### Triage, which is where the time actually goes

Running the agents is quick; deciding what is true is not. Every finding is dispositioned in writing
as **accept** (with the fix), **reject** (with the reason), or **defer** (roadmapped). Two rules from
experience on this project:

- Sub-agents here have produced **contradictory reports about the same test suite** when run
  concurrently, because they observe each other's mid-edit state. Nothing they claim about repo state
  is trusted without me reproducing it.
- A confident finding is not a correct one. Anything substantive gets verified against the code
  before it becomes a task.

**Exit:** every finding dispositioned; accepted ones become tasks in Phases 2 and 3; the log records
the rejected ones with reasons, so they are not re-raised next cycle.

### A second, smaller pass at the end

Phase 5 re-runs a single reviewer against the *final* state as an independent audit, paired with the
Phase 0 matrix re-run. Cheap, and it catches anything the cleanup broke or any claim that drifted.

---

## Phase 1 — Documentation split by audience (1–2 days)

The problem is not that the docs are bad, it is that they are written for the two people who built
this. A new user does not care which bug cost us an afternoon.

### 1a. README rewrite, modelled on the 2023 layout

The 2023 README worked because it answered questions in the order a user asks them. Adopt that
shape, drop its Docker section (removed in D10), and keep our stronger content.

Target structure, in order:

```
Title + one-paragraph what-it-is
Table of contents
System requirements
Installation
Running the simulation          <- shortest path to a moving picture
Features                        <- one subsection per feature, each with copy-paste usage
  Camera and pose input
  Distance sensor
  Bounding boxes
  Gimbal
  Frame capture
  RTSP video
  Multiple vehicles
  Recording
ROS 2 topics: inputs and outputs   <- one table each
Conventions (frames, angles, units, packet format)
Configuration reference
Scripting with the devkit
Debug tools
Troubleshooting
Contributing / development
License
```

Rules for the rewrite:

- Every feature subsection answers: what it does, how to turn it on, how to use it, how to confirm
  it is working. In that order, with a runnable command or snippet.
- No history. "This differs from the previous generation because…" belongs in a migration note at
  the bottom, once, not scattered through.
- No war stories. The reasoning that made a design correct stays in the code comment or the
  development log, not in user documentation.
- Current README is 746 lines across 19 top-level sections. Target roughly the same length but with
  the weight moved from troubleshooting and architecture into features and usage.

### 1b. Separate internal from user-facing docs

- `docs/development-log.md` (3,372 lines) is an engineering log. Valuable, but it is the single biggest source of
  "written for us" material and it sits at the repo root where a newcomer opens it first. Move to
  `docs/development-log.md`, keep appending, and stop linking it from the README's main flow.
- `docs/dev/roadmap.md` (453 lines) is planning. Move to `docs/dev/roadmap.md`.
- `docs/dev/usd_build_sheet.md` is a task list for one person. Move to `docs/dev/usd_authoring.md` and
  rewrite as *how to author a layer*, which is genuinely useful to a user adding a feature.
- Add `docs/architecture.md`: pull the architecture section out of the README so the README stays
  about using the thing.

**Exit:** README contains no proper nouns of team members, no bug archaeology, and no section that
exists only because we found it interesting. A reader who has never seen the repo can install, run,
and use every feature from the README alone.

---

## Phase 2 — Source cleanup (2–3 days)

Overall comment density is 3.6% (549 of 15,197 lines), which is healthy. The problem is
concentrated and stylistic rather than volumetric.

### 2a. Comment style pass

Measured offenders: `config/schema.py` at **16.8%** comments (120 of 716 lines),
`sim/__main__.py` at 12.8%. Across `src/` there are 23 references to "2023", 6 "used to",
5 "the old repo", 3 "previously".

Policy to apply:

- A comment says **why**, in one or two lines. If it needs a paragraph, the code needs a better name
  or a docstring.
- No comparisons to the previous generation in source. Where a convention exists *because* 2023 did
  it differently, state the convention, not the history.
- No incident reports. "This aborted the process and took three rounds to find" becomes "changing
  this on a live graph aborts Kit".
- Keep every comment that prevents a reader from *undoing* a non-obvious decision. Those earn their
  space and must survive the pass.

### 2b. Duplicate logic sweep

Candidates to examine, not yet confirmed:

- Vehicle/camera resolution helpers in `configurator.py` (`_first_vehicle_id`, `_first_camera_id`,
  `_camera_for`) against `config.schema` accessors — likely overlapping responsibility.
- Topic derivation across `_resolve_image_topic`, `_resolve_distance_topic`,
  `_resolve_vehicle_topic` — three functions with one shape.
- Quaternion/matrix helpers in `geo/rotations.py` versus the ad-hoc conversions inside OGN nodes.
- `_pump`/settle-frame patterns repeated in `runtime.py`.

**Exit:** each candidate is either unified or has a one-line comment explaining why the duplication
is deliberate.

### 2c. Dead code and dead surface

The M8 guards already cover dead config keys (zero), unregistered `Method` members, unreachable
handlers, and public symbols without consumers. Extend to:

- Unused `.ogn` inputs/outputs on our own nodes.
- Config fields read but never *acted on* (a stricter test than "referenced somewhere").
- Test helpers duplicated across test modules.
- `scripts/` — `crash_rate.sh` is a debugging tool from a specific investigation; decide keep or cut.

### 2d. Error message audit

Every raised error a user can hit should name the thing that is wrong and the action that fixes it.
We have good examples (the georeference mismatch names both values, the distance, and three
remedies). Audit the rest against that bar.

---

## Phase 3 — Verify every feature (2–3 days)

The core of what you asked for. Work the Phase 0 matrix top to bottom; nothing is "probably fine".

### 3a. Bucket A — automated

Fill gaps found in Phase 0. Two rules learned the hard way this month:

- Assert on **observable outcomes**, not on the formula that produces them. The gimbal axis bug
  passed every matrix-multiplication test while pitch came out as roll; only asserting the camera's
  world look direction caught it.
- Guard the **wiring**, not just the logic. Three separate bugs were declared-but-unconnected
  inputs that silently fell back. Those guards now exist; extend them to any new input type.

### 3b. Bucket B — live-scripted

Extend `scripts/eyes_on_check.py` into a `--verify` mode that **asserts** rather than describes:
launch headless, drive the feature, check numbers, report pass/fail, exit non-zero on failure.

Verification protocol, written down because I got this wrong repeatedly:

1. Kill stray Isaac processes **before** launching, not only after. Orphans hold GPU memory and make
   results meaningless.
2. Wait for `get_state().ready`, never just an open port.
3. Allow for DDS discovery before concluding a topic is silent.
4. Use `python.sh -m isaac_core.sim`; an in-process probe that opens a stage after app load hangs.
5. Every Isaac command gets `timeout -k 5 N` — Isaac ignores SIGTERM, so plain `timeout` never
   returns.

Numbers to assert, all previously measured: pose translate = altitude − ENU reference; distance
sensor = ground clearance when boresighted down; bbox arrays index-aligned with real pixel boxes;
capture at requested resolution; RTSP `DESCRIBE` returns H.264 SDP; per-vehicle topics namespaced.

### 3c. Bucket C — eyes-on

Keep the eight scenarios, add any feature the matrix shows is unrepresented, and record a
pass/fail sheet per release rather than relying on memory.

**Exit:** every matrix row has a dated result. Anything failing is fixed, or moved to
"not in this version" in the README with a reason.

---

## Phase 4 — Repo hygiene (half a day)

- `.gitignore` for `__pycache__`, `*.egg-info`, generated OGN caches; confirm nothing generated is
  tracked.
- `pyproject.toml`: real description, keywords, classifiers; confirm the console scripts list is
  exactly the shipped tools.
- `config/default.toml` read end to end as a reference document, since that is what users will do.
- Delete stale artefacts: `src/isaac_core.egg-info`, leftover `__pycache__` for deleted modules.
- No `CONTRIBUTING.md` (Ofer's call). House rules stay in `docs/development-log.md`.

---

## Phase 5 — Release gate (half a day)

A single checklist that must pass before calling V2 done:

- [ ] Full suite green; all hooks green; 23 import-linter contracts kept.
- [ ] Feature matrix: every row dated and passing (re-run, not the Phase 0 run).
- [ ] Every Phase 0.5 finding dispositioned; no accepted finding left unimplemented.
- [ ] README: install → run → every feature, followed with no outside knowledge.
- [ ] No proper nouns of team members outside the development log.
- [ ] Dead config keys: zero. Dead public symbols: zero. Unregistered methods: zero.
- [ ] Every `NotImplementedError` is listed in the README as not-in-this-version, with a reason.
- [ ] Fresh-clone check on a **different machine with a fresh Isaac Sim install**: install per the
      README and fly, with no knowledge from this one.
- [ ] Python 3.10 and 3.12 both green.

---

## Sequencing and effort

| Phase | Work | Size | Owner |
|---|---|---|---|
| 0 | Feature matrix | 0.5 d | Kiro |
| 0.5 | Adversarial agent review + triage | 1 d | Kiro (Ofer sees the findings) |
| 1 | Docs split + README rewrite | 1–2 d | Kiro, review by Ofer |
| 2 | Source cleanup | 2–3 d | Kiro |
| 3 | Verify every feature | 2–3 d | Kiro (A/B), Ofer (C) |
| 4 | Repo hygiene | 0.5 d | Kiro |
| 5 | Release gate + **re-run the Phase 0 matrix** | 0.5 d | both |

Phase 0.5 runs after 0 and before 1, for the reasons given in that section. Phase 3 must come
after 2, so cleanup cannot silently break something the matrix then certifies as working.

**Needs Ofer:** README review (voice and accuracy), eyes-on scenarios, screenshots, and the call on
the two deferred methods below.

---

## The two deferred methods

`features.enable/disable(...)` and `load_scene(...)` are registered on the control plane and raise
`NotImplementedError` with an explanation. Everything else the devkit exposes works.

**What they would do**

- `features.enable("bbox")` — compose a feature layer onto the **running** stage, so you could turn
  bounding boxes on mid-flight. Today features are chosen in config before launch.
- `load_scene("other")` — swap the entire scene while running. Today the scene is chosen in config
  before launch.

**Why they are not implemented**

Both are stage-lifecycle operations: adding a USD reference to a live stage, or closing a stage that
Cesium, the ROS 2 bridge and several OmniGraph graphs all hold references to. That is the most
dangerous area in this codebase, and the evidence is specific:

- The startup segfault needed 60 warm-up frames before *any* stage operation; a 30-frame probe
  looked fine and was wrong.
- Writing a relationship onto a live OmniGraph node aborts Kit outright — exit 0, no traceback.
- Renaming an OGN attribute leaves stale authored attributes that silently prevent a node from
  instantiating.

In every case the failure mode was a silent abort rather than an error, which is the worst kind to
ship.

**Why it costs users little**

Neither blocks a workflow. Features are declared in config; changing scene means a restart, which is
about fifteen seconds. Nothing in the 2023 repo did either, so this is not a regression.

**Recommendation: keep both deferred for V2.** Ship them raising a clear error, listed in the README
under what this version does not do, and revisit in V3 where live stage composition can be designed
deliberately with a crash-rate harness rather than bolted on at the end. The alternative — implement
them now — spends the riskiest days of the project on capability nobody has asked for, immediately
after a cleanup pass, and the guard test already prevents the README from claiming they work.
