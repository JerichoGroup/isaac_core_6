# Phase 8 — the endgame before 1.0.0

No new features. Validate, retest, clean, then ship as `1.0.0`.

Worked **strictly in order**. Each stage finishes before the next begins, so a stage's findings land against
a repo that can still be retested and so no stage has to be held in mind while another runs.

---

## Stage 0 — the harness blocker — **DONE**

**Framing, corrected.** This has never been seen in normal use. It appears only when the **full system
suite is run repeatedly**: every module passes on its own, a full run from a genuinely clean machine passes
in about 182 s, and a later run fails its first module or stalls past a 40-minute timeout. So this is a
*test-harness* reliability problem, not a product defect, and the fix is expected in `tests/system/` rather
than in `src`. Saying otherwise overstates it.

It still has to be dealt with first, because the system suite is the instrument every later stage relies on.
A suite that cannot be run twice cannot certify anything.

**Method: find the root cause, fix only that.** No refactoring, no opportunistic cleanup. The first job is a
*cheap* reliable reproduction — the current one costs 20–40 minutes, which is too slow to bisect against.

Hypotheses, cheapest first:

1. **State accumulating across launches inside one pytest process.** Four module-scoped fixtures launch four
   simulators sequentially in one process. Test by running each module in its own process and comparing.
2. **Shutdown not actually complete before the next launch.** `_await_gpu_release` waits; it does not verify.
   Instrument it with real GPU readings.
3. **Kit state outside the Cesium cache** in `~/.cache/ov` or `~/.local/share/ov`. Bisect by clearing subsets.
4. **A per-machine launch budget.** This machine segfaults roughly one launch in three; the second machine
   never does. Compare there.

**Outcome, as it turned out.** Neither a leak nor accumulation nor a launch failure — all ruled out by
measurement. On a two-vehicle Cesium stage the simulation thread sometimes stops servicing main-thread calls
while `get_state` keeps answering, because Kit's `update_app()` does not return; two viewports streaming
tiles is a known main-thread hog. That is inside Kit, so there is no honest product fix.

What *was* fixable was mine: the readiness check counted attempts instead of using a wall-clock deadline, so
it inherited the 240 s call budget and turned a documented 20-second fail-fast into 40 minutes; and nothing
verified the simulation thread was turning, so every test then burned the full call budget. Both fixed in
`tests/system/conftest.py`, no product changes.

Measured after: two consecutive full runs at 179 s (17 passed, 10 skipped fast and accurately) and 216 s
(27 passed). Both previously hit a 2600-second timeout. Single-vehicle use was never affected, which is why
this never appeared in normal use.

---

## Stage 1 — sub-agent audit army

Eight auditors with narrow, non-overlapping briefs. **Each returns findings with evidence and changes
nothing.** Every finding is then verified independently before anything is acted on: a sub-agent's claim is
a lead, not evidence. This keeps them to external audit, the only role they are trusted with.

| Auditor | Brief | Output |
|---|---|---|
| Documentation truth | Every factual claim in `README.md` and `docs/*.md` checked against code and behaviour | False or stale claims, ranked |
| Trap hunter | Tests that would still pass with the feature broken: mocks where the real path matters, endpoint-only assertions, one dimension exercised with the rest at defaults | Weak tests, with why |
| Error-message quality | Every raise and error path: does it say what broke, where, and what to do next? | Unhelpful messages |
| API consistency | Naming, argument order, defaults and return shapes across devkit, CLI and control plane | Inconsistencies |
| Dead code and config | Unreachable code, unused exports, config keys that change nothing, stale exemption lists inside guards | Removal candidates |
| Security and robustness | Path confinement, unauthenticated network surface, untrusted input, cleanup on failure | Risks by severity |
| Fresh-eyes onboarding | Follow the README as a new user with no context; report every confusion and breakage | Friction log |
| Convention conformance | No archaeology in `src`, PEP 257 first-line summaries, comment style, no section banners, flat test functions, zero suppressions in `src`/`scripts` | Violations |

---

## Stage 2 — harsh testing

### 2.1 The untested paths — **DONE**

Shipping a feature that has never been run is not acceptable for 1.0.0. All three are testable here and now.

- **`pose_source = "ros"`** — the second of the two advertised pose sources. Never driven live. Test by
  publishing MAVROS-shaped `NavSatFix` + `PoseStamped` from system Python with `rclpy`.
- **`isaac-core-mavlink`** — the MAVLink bridge. Unit-tested only. Test against a real `pymavlink`
  connection emitting `GLOBAL_POSITION_INT` and `ATTITUDE`.
- **`TopicRecorder` and its factories** — `video_recorder`, `pose_recorder`, `range_recorder`,
  `bbox_recorder`. Unit-tested only. Test by recording a real flight to disk and reading the files back.

Also never exercised live: `isaac-core-inspect --poll`, and the GUI with an actual window.

### 2.2 Combinations, not dimensions — **DONE**

The trap that let six bugs ship was exercising one axis with the others at defaults. A covering design over
the axes that genuinely interact — `{1,2} vehicles`, `{udp,ros}` pose source, `{default,pinned}` ports,
`{gimbal,zoom,both}`, `{headless,GUI}`, `{earth,house}` scene — hitting every pair at least once.

### 2.3 Adversarial config — **DONE**

Every schema key: below range, above range, wrong type, empty, unicode, path traversal, colliding with
another vehicle. Each rejection must name the key and say what to do instead.

### 2.4 Resource and lifecycle — **DONE**

Process, port, GPU and file-handle checks after each of: clean exit, exception, Ctrl-C, SIGKILL of the
parent, and a failed launch. Plus a soak run, since the feature matrix currently states outright that
sustained-load behaviour is uncovered.

### 2.5 Fresh clone on the second machine — **READY TO RUN**, see `docs/dev/phase_8_second_machine.md`

Prepared as a script for Ofer to run on the 6.1.0-rc.26 box, with an exact list of what to report back. This
is the only test of the install path and of 6.1 compatibility.

---

## Stage 3 — eyes-on scenarios — **DONE, 16 scenarios ready to run**

Every shipped capability validated by eye, because that is the only way some of them can be. Each scenario
writes an artefact to `~/isaac_core_out/` and comes with **what you should see** and **what broken looks
like**, so a pass is a judgement against a stated expectation rather than a vibe.

| # | Scenario | Capabilities covered |
|---|---|---|
| 1 | Terrain flight over `earth.usda`, gimbal aim, zoom sweep | camera, UDP pose, Cesium terrain, gimbal, zoom |
| 2 | ROS pose source driven by MAVROS-shaped topics | `pose_source = "ros"`, the camera_ros layer |
| 3 | Bbox over `earth.usda`'s labelled cubes | bbox, occlusion, geodetic target positions |
| 4 | Segmentation over Cesium terrain | the documented terrain caveat, shown honestly |
| 5 | Two vehicles: independent gimbal, zoom, two RTSP streams | swarm parity, topic and mount namespacing |
| 6 | `PoseBot` library: orbit, path, steer, track, follow | flight generators, tracking, follow-me |
| 7 | Frame capture at native resolution and 4K | capture, resolution independence |
| 8 | GUI pose sender flown by hand, two tabs | GUI, readback line, jog keys, stream view |
| 9 | MAVLink bridge driving the simulator | the MAVLink adapter |
| 10 | Distance sensor against known geometry | rangefinder, boresight with the camera |
| 11 | Recording a flight to disk and playing it back | `TopicRecorder`, video and bbox recorders |
| 12 | A user-authored layer from outside the repo | layer authoring, entry points, search paths |

Delivered as scenarios 9-16 appended to `scripts/eyes_on_check.py`, alongside the eight that already
existed. All sixteen smoke-tested end to end. The reviewer's guide, with what to look for and what to report
back, is `docs/dev/phase_8_eyes_on.md`.

More scenarios are better than fewer. Anything shipped and not on this list is a gap in the list.

---

## Stage 4 — cleaning, and deletions last

- Version to `1.0.0`.
- Final feature-matrix pass: it is the certifying document, and its counts are now guarded by a test.
- Archaeology sweep in `src`.
- Dead code removed, from Stage 1's findings and verified independently.
- README accuracy pass.
- **Deletions last, once nothing else needs them.** `v2_finalization_plan.md` and `v3_plan.md` are gone.
  The eyes-on guide and the second-machine guide are kept: both are tools rather than scaffolding.

No changelog, no tagging ceremony. Nothing is committed or pushed by me.
