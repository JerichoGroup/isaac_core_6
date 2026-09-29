# Finishing V3 — remaining task list

Written as a handover. Assume the reader has no memory of this work: everything needed to finish is here or
pointed at from here. Tasks are ordered so each one can be verified before the next begins.

---

## 1. Where the repo stands

Verified at the time of writing, not remembered:

| | |
|---|---|
| Version in `pyproject.toml` | **`1.0.0`** |
| Unit tests | **2201 passed, 28 skipped** |
| Pre-commit hooks | **all 16 pass** |
| Import-linter contracts | **20 kept, 0 broken** |
| Lint suppressions in `src`/`scripts` | **0** |
| System tests | **27 pass** from a clean machine, about 3 minutes |
| Eyes-on scenarios | **16**, all smoke-tested, `scripts/eyes_on_check.py` |
| Branch | `dev`, **38 uncommitted files** |

Phases 1–7 are complete. Phase 8 is the endgame: validate, retest, clean, ship as 1.0.0. No new features.

Plan and progress live in `docs/dev/phase_8_plan.md`. Stage 0, 1, 2.1, 2.2, 2.3, 2.4 and 3 are done.
Stage 2.5 is ready but needs a human. Stage 4 is the remaining work and is listed below.

The engineering log, `docs/development-log.md`, is the reasoning record. It is long, and it is the thing to
read when a decision looks arbitrary — most of them are not.

---

## 2. Hard rules — violating any of these is a failure

These are the repo owner's standing instructions. They are not stylistic preferences.

1. **Never run a writing git command.** No `add`, `commit`, `push`, `rm`, `mv`, `checkout -f`, no amend, no
   tag. Read-only git is fine. Prefer plain `mv`/`rm` over `git mv`/`git rm`, which dirty the index. The owner
   commits.
2. **Never install anything.** No `pip install`, `apt`, venv creation. Report a needed dependency instead.
3. **Never author or edit `.usda`/`.usd` files.** The owner authors all USD. You may write build sheets and
   `layer.toml` manifests. A layer that contributes behaviour rather than prims needs no USD at all — omit the
   `usd` key, as `src/isaac_core/assets/layers/segmentation/layer.toml` does.
4. **`/home/ofer/clones/isaac_core_2023` and `/home/ofer/isaacsim` are read-only.**
5. **Sub-agents only for external audits, never for feature work.** Their findings are leads, not evidence:
   verify every one yourself before acting. Stage 1 did this and one HIGH finding turned out overstated.
6. **Prefix every Isaac command with `timeout -k 5 N`.** Isaac ignores SIGTERM, so plain `kill` and plain
   `timeout` do not stop it.
7. **Never `pkill -f`/`pgrep -f` with a pattern that matches your own command line.** Walk `/proc` excluding
   your own PID and your parent's. Kill strays *before* launching, not only after.
8. **Ask before deleting anything** not already sanctioned.
9. **Correct the owner when he is wrong, and report honestly.** Overclaiming is worse than a gap.

### Style rules that are enforced or expected

- **No lint suppressions in `src`/`scripts`** (`# noqa`, `# type: ignore`). Allowed in `tests/` and
  `extensions/`, but never *stale* ones — RUF100 is on and will fail the build.
- **No section-banner comments** like `# ===== Section ===== #` or `# ----- thing ----- #`. Short one-line
  comments are preferred. Large comment blocks are the problem, not density.
- **No archaeology in `src`**: no "2023", "previous generation", "legacy", "used to be". State the constraint
  positively. `docs/` may discuss history.
- **Docstrings: PEP 257, summary on the FIRST line** (D212 enforced, D213 ignored).
- **Tests are flat module-level functions, never classes.** Every test annotated `-> None`. A `#` comment
  where a test guards a past bug.
- **Never bind a hardcoded port in a test** — bind 0 and read it back.
- **README**: at most one emoji per section heading; every requirement/feature/troubleshooting entry inside a
  `<details>` block.
- **Never ship a feature that claims to work but does not.**

---

## 3. Verification recipe

Run the first three after every change. Run the rest before declaring anything finished.

```bash
cd /home/ofer/clones/isaac_core_6

# 1. Unit suite — expect 2195 passed, 28 skipped
timeout -k 10 600 python3 -m pytest -q

# 2. All 16 pre-commit hooks. mypy needs --files because of a per-hook exclude.
FILES=$( { find src tests scripts -name '*.py'; } | sort -u | tr '\n' ' ')
for h in check-added-large-files check-merge-conflict end-of-file-fixer trailing-whitespace \
         check-yaml check-json check-toml check-xml check-case-conflict check-docstring-first \
         check-executables-have-shebangs check-shebang-scripts-are-executable ruff ruff-format \
         import-linter; do
  timeout -k 5 300 pre-commit run $h --all-files >/dev/null 2>&1 || echo "FAIL: $h"
done
timeout -k 5 300 pre-commit run mypy --files $FILES

# 3. Layering contracts — expect 20 kept, 0 broken
timeout -k 5 120 env PYTHONPATH=src lint-imports

# 4. System tests. Free the GPU first (see traps) and clear the Cesium WAL.
rm -f ~/.cache/ov/cesium-request-cache.sqlite-wal ~/.cache/ov/cesium-request-cache.sqlite-shm
rm -rf /tmp/carb.*
timeout -k 40 2600 python3 -m pytest tests/system -q --system     # expect 27 passed, ~3 min
```

`end-of-file-fixer` and `ruff-format` **rewrite files**, so a first run can report failure and a second pass.
Run them twice before believing a failure.

Kill strays between Isaac runs:

```bash
python3 - <<'PY'
import os, signal, time
from pathlib import Path
me = {os.getpid(), os.getppid()}
def find():
    out = []
    for e in Path("/proc").iterdir():
        if not e.name.isdigit() or int(e.name) in me:
            continue
        try:
            j = b" ".join((e / "cmdline").read_bytes().split(b"\0")).decode(errors="replace")
        except OSError:
            continue
        if "telemetry" in j:
            continue
        if "isaac_core.sim" in j or "kit/kit" in j:
            out.append(int(e.name))
    return out
for sig in (signal.SIGTERM, signal.SIGKILL):
    for pid in find():
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass
    time.sleep(3)
print("strays:", len(find()))
PY
```

---

## 4. The tasks

Each is small and independently verifiable. Order matters only where stated.

### T1 — Remove two dead symbols — **DONE**

Both confirmed to have exactly one reference: their own definition.

- `REQUIRED_EXTENSIONS` — `src/isaac_core/sim/runtime.py:143`. A tuple never read; the similarly named
  `_enable_required_extensions()` iterates `self._config.sim.extensions` instead.
- `VehicleState.distance_to` — `src/isaac_core/vehicle/state.py:225`. No caller anywhere; motion code uses
  the helpers in `src/isaac_core/geo/distance.py`.

Neither is caught by the existing dead-code guard, which scans public top-level `class`/`def` but not module
constants or methods. Consider whether widening `tests/unit/test_no_dead_code.py` to cover both is worth it —
if you do, expect it to surface more candidates that need judgement.

**Verify:** `grep -rn 'REQUIRED_EXTENSIONS\|distance_to' src/ tests/ scripts/ --include='*.py'` returns
nothing, then the unit suite.

### T2 — Remove archaeology from `src` and `scripts` — **DONE**

Rule 3 above. Nine sites, all confirmed:

| File:line | What to remove |
|---|---|
| `src/isaac_core/install.py:4` | "pointing at the old 2023.1.1 install" |
| `src/isaac_core/install.py:41` | "Year-based versions (2022, 2023, etc.) predate the 6.x numbering scheme" |
| `src/isaac_core/install.py:130` | "Year-based versions (e.g. 2023.1.1) predate the 6.x numbering" |
| `src/isaac_core/install.py:138` | "used the old namespace" |
| `src/isaac_core/install.py:173` | "a leftover 2023.1.1" |
| `src/isaac_core/sim/composer.py:176` | "a plan that predates per-vehicle planning" |
| `src/isaac_core/sim/composer.py:561` | "Only a *mismatch* used to be reported" |
| `src/isaac_core/config/schema.py:397` | "It used to be a dict keyed by a name" |
| `scripts/send_test_pose.py:40` | "Matches the previous generation's senders" |

`src/isaac_core/devkit/bot.py:214` says "where the target used to be relative to the aircraft" — that
describes geometry during a manoeuvre, not repo history. Leave it.

The install.py cases need care: the *code* genuinely must reject year-based versions, so the constraint stays
and only the narration goes. "Year-based versions predate the 6.x scheme" becomes something like "Only 6.x
and newer are supported; a year-based version string is rejected."

**Dependency worth knowing:** `tests/unit/test_install.py` and `tests/unit/test_install_resolution.py` both
reference 2023-era version strings — `test_version_2023_parsed`, `test_is_supported_false_for_2023` — because
rejecting those versions is real behaviour that must keep working. Removing the *narration* from `install.py`
must not change what the code rejects, so run those two files specifically after editing.

**Do not "clean" those test names.** Rule 3 covers `src/` and `scripts/` only. A test that names the thing it
proves is rejected is describing behaviour, not narrating history, and renaming it would make the suite say
less.

**There is no archaeology guard.** Nothing enforces rule 3, which is why these nine sites accumulated.
Adding `tests/unit/test_no_archaeology.py` that walks `src/` and `scripts/` with a named exemption list would
stop it recurring, and is worth the twenty lines. If you add it, the `bot.py:214` line needs an exemption with
a reason, not a silent skip.

**Verify:** `grep -rniE '2023|previous generation|legacy|predate|used to be' src/ scripts/ --include='*.py'`
returns only the `bot.py` line; then
`timeout -k 10 300 python3 -m pytest tests/unit/test_install.py tests/unit/test_install_resolution.py -q`;
then the full unit suite.

### T3 — Remove six banner comments — **DONE**

- `src/isaac_core/devkit/recording.py:397,400` and `586,588`
- `src/isaac_core/config/loader.py:257,259`

Delete the rule lines. Keep any words on the line between them as a plain one-line comment if they say
something useful.

**Verify:** `grep -rnE '^\s*# -----|^\s*# =====' src/ --include='*.py'` returns nothing.

### T4 — Fix three silent failures — **DONE**

Found by the Stage 1 error-message audit and verified. Each turns a diagnosable failure into silence.

1. **`src/isaac_core/debug/pose_sender_gui.py:505`** (the bare `except` is at 521) — `readback_vehicle_names()` catches every exception and
   returns `()` with no log. A failed config read then looks identical to "the simulator has zero vehicles".
   Log at `warning` with `exc_info=True`.
2. **`src/isaac_core/sim/composer.py:635`** — a `sim.viewport_camera` naming a prim that is not
   on the stage falls back to a type search and reports the miss only at `debug`, which the default level
   hides. The user sees a working-but-wrong camera with no reason. Raise it to `warning` and name the
   configured value.
3. **`src/isaac_core/sim/runtime.py:2017`** — `_viewport_resolution()` catches everything and
   returns `(0, 0)`, which then flows into capture and restore logic as though it were real. Log it.

**Verify:** unit suite, plus run eyes-on scenario 5 (capture) to confirm nothing regressed.

### T5 — Cap the client's read buffer — **DONE**

`src/isaac_core/control/client.py:148-149` reads in a loop until it sees a newline, with no size limit. The
*server* caps a request line at `MAX_REQUEST_BYTES = 1 << 20` (`src/isaac_core/control/server.py:50,244`).
The client should mirror that: a hostile or broken server that never sends a newline currently makes the
client grow until it dies.

Low severity — it needs a malicious or broken server — but it is a two-line asymmetry in a security-sensitive
place, and the fix is obvious.

**Verify:** add a test that a server sending a large newline-free stream raises rather than exhausting memory.
Bind port 0 in that test, never a literal.

### T6 — Version to 1.0.0 — **DONE**

`pyproject.toml:3`. Do this **after** T1–T5, so the version bump is the last code change.

**Verify:** `grep -n '^version' pyproject.toml`, then the full recipe in section 3 including system tests.

### T7 — Final feature-matrix pass — **DONE**

`docs/dev/feature_matrix.md` is the certifying document: it is what a reader consults to learn what works.
`tests/unit/test_feature_matrix_claims.py` already guards its counts, that every cited file exists, and that
no row claims "pass" for something `docs/dev/roadmap.md` calls unbuilt. Those guards will fail the build if a
count drifts, so trust them but read the prose too.

Check specifically:
- Every row's evidence still exists and still exercises the row's claim. **A citation is not coverage** — the
  zoom row once cited a file with no reference to the runtime at all.
- The counts match after T1–T6.
- Nothing shipped is missing a row. Anything present and unrowed is a gap in the matrix.

### T8 — README accuracy pass — **DONE**

The Stage 1 documentation audit found and fixed six false claims. Do a final read for:
- Any documented signature, flag, config key, topic, port or prim path that no longer matches `src`.
- Any "it does X" that is not true after T1–T7.
- The `<details>` and emoji conventions in section 2.

`tests/unit/test_readme_honesty.py` exists and covers some of this. It is not exhaustive.

### T9 — Deletions, last — **PARTLY DONE**

The owner's instruction: deletions are the final act. Once nothing else needs them:

- ~~`docs/dev/v2_finalization_plan.md`~~ — deleted.
- ~~`docs/dev/v3_plan.md`~~ — deleted.
- `docs/dev/phase_8_plan.md`, `docs/dev/phase_8_eyes_on.md`, `docs/dev/phase_8_second_machine.md`, and this
  file — **ask the owner.** The eyes-on guide has ongoing value as a manual test plan; the others are
  scaffolding. He deleted the equivalent V2 documents (`style_questions.md`, `phase_0_5_triage.md`) at this
  point, so the precedent is to remove scaffolding and keep tools.

Keep `docs/dev/roadmap.md` and `docs/migrating_from_2023.md`: they describe what the previous generation did
and why things are shaped as they are. Deleting history to tidy a diff loses the reasoning.

**Verify:** after deleting, `grep -rn 'v3_plan\|v2_finalization' . --include='*.md' --include='*.py'` finds no
dangling references, then the full recipe.

---

## 5. Needs a human

### H1 — Second machine (Stage 2.5)

Everything is ready: `scripts/fresh_clone_check.sh` and `docs/dev/phase_8_second_machine.md`.

**Blocked on the owner pushing `dev`.** The script clones from the remote, so it verifies whatever is on the
branch — not the working tree, which currently has 38 uncommitted files. The script checks for files that only
exist in the V3 work and stops if the branch is stale, because a green report from the wrong branch is worse
than a failure.

This is the only test of the install path and of Isaac Sim 6.1 compatibility. Everything else was verified on
one machine against 6.0.1-rc.7.

### H2 — Decision: the SIGKILL leak — **DECIDED: leave it**

Measured in Stage 2.4: four of five exit paths leave nothing behind — clean exit, exception, Ctrl-C and a
failed launch are all clean. **`kill -9` of the owning process leaks 2 processes, all three ports, and about
2.6 GiB of GPU.**

That is inherent: no cleanup runs after SIGKILL, and `Sim.launch` uses `start_new_session=True` deliberately
so signals aimed at a caller do not tear the simulator down mid-frame. `prctl(PR_SET_PDEATHSIG)` would make
the simulator die with its parent but pulls against that isolation. **Decided: leave it as is.** `prctl` would trade a reliable property (signals aimed at a caller never tear
the simulator down mid-frame) for protection against a case the user caused deliberately with `kill -9`, and
every *normal* exit is already clean. The README troubleshooting entry documents the cause, the cost and the
recovery, which is the right level of response to a self-inflicted hard kill.

---

## 6. Known open issues — document, do not chase

### The intermittent two-vehicle timeout

On a two-vehicle Cesium stage the simulation thread sometimes stops servicing main-thread calls: `get_state`
keeps answering because the control-plane thread handles it, while `step` and `get_pose` all time out
identically. Our loop is `update_app()` then drain queued tasks, so if Kit's `update_app()` does not return,
nothing drains.

Frequency observed: 3 of 4 two-vehicle combinations pass; the swarm system module passes 10/10 in isolation
and occasionally skips in a full run. The harness detects it with one trivial `step(1)` probe and skips fast
with an accurate message rather than stalling — see `_main_thread_turning` in `tests/system/conftest.py`.

**Two full days went into this. Read the "Stage 0, settled" entry in the development log before touching it.**
The attribution was wrong twice: first blamed on Kit, then on GPU contention, and the honest position is that
a harness defect accounted for the 40-minute stalls while GPU contention accounted for most of the rest. Do
not re-open it without new evidence.

### Segmentation on Cesium terrain

By design. Streamed tiles are regenerated geometry with no durable per-prim identity, so the same patch of
terrain can change colour **between frames within one recording**, not merely lack per-building instances.
Authored objects segment cleanly. Judge colour stability on authored geometry only. Already documented in the
README and in the eyes-on guide.

### mp4 is lossy

A recording whose annotator produced 27 flat colours decodes to over 22,000 because compression stipples every
region edge. Fine for looking at; useless for extracting masks. Documented.

---

## 7. Traps that will cost you hours

Hard-won. Each of these was learned the expensive way.

1. **Free the GPU before any Isaac work.** A Cesium stage needs about 3 GiB; two vehicles need more. A
   resident language model starves the renderer *silently* — the stage composes and then stops responding with
   nothing saying why. Check `nvidia-smi --query-gpu=memory.free --format=csv,noheader` and want > 4000 MiB.
   **A resource measured only while the system under test is idle is not a baseline.**
2. **Kill strays before launching.** Isaac ignores SIGTERM. A lingering simulator holds the control port, UDP
   33333 and RTSP 8554; the next run composes happily and then sits at a pose of zero while the real cause is
   one `Address already in use` line buried in its log.
3. **Never PIPE Isaac's stdout without draining it.** It fills the 64 KiB pipe buffer and blocks forever.
   Redirect to a file.
4. **Bound work by a deadline, not by attempts.** `attempts × slowest-possible-call` is the real budget. A
   readiness check written as 10 attempts became a 40-minute hang when an unrelated call timeout was raised to
   240 s.
5. **Make a slow intermittent failure cheap to observe before investigating it.** Bisecting against a
   40-minute failure is hopeless. A 90-second reproduction found in minutes what days of full-suite runs did
   not.
6. **A probe that swallows its own exception sends you hunting the wrong thing.** Print the reason.
7. **Clear `__pycache__` after break-and-restore experiments.** An edit that preserves file size defeats
   invalidation.
8. **The Cesium write-ahead log stalls startup at any size.** Clear
   `~/.cache/ov/cesium-request-cache.sqlite-{wal,shm}` before launching. 763 MiB and 23 GiB both did it.
9. **Editing multi-line imports with `str.replace` produces broken syntax.** Rebuild the block.
10. **Never use truncated error text as a `str.replace` target.** It once matched a substring and corrupted a
    document.
11. **Test the shipped artefact, not the generator**, and test **combinations**, not one axis with the rest at
    defaults. Six real bugs shipped past a green suite that way. The pattern: *"I verified the thing I had just
    built, in the shape I had just built it, rather than the thing a user touches."*
12. **Prove a guard works by injecting the original offence and watching it fail.** Several "guards" in this
    repo's history were vacuous until this was done.

### Environment facts

- Isaac Sim **6.0.1-rc.7** at `/home/ofer/isaacsim`; the second machine runs **6.1.0-rc.26**.
- Isaac's Python is **3.12** and has `cv2`, `PIL`, `numpy`. System Python is **3.10**.
- `tomllib` is not on 3.10 — use `tomli`.
- `rclpy` **cannot** be imported inside Isaac; it works on the host. Use
  `export PYTHONPATH="src:${PYTHONPATH}"` — overriding wipes ROS's own path.
- ROS 2 Humble, `ROS_DOMAIN_ID=13`, `DISPLAY=:1` works for GUI runs.
- ENU reference: lat 32.22481, lon 35.25621, alt 516.7. Stage `translate z = altitude − 516.7`.
- Startup segfaults roughly 1 launch in 3 on the first machine, never on the second. Kit-internal; reproduces
  with our extensions disabled.
- The house test scene is `/home/ofer/Documents/tmp_usd/house.usda` — non-Cesium, same georeference as
  `earth.usda`. Framing that works: `lat 32.224800, lon 35.256190, alt 520.0, pitch −25, yaw 20`.

---

## 8. Definition of done

1. T1–T9 complete, each verified.
2. Full recipe in section 3 green: 2195+ unit tests, all 16 hooks, 20 contracts, 0 suppressions, 27 system
   tests.
3. All 16 eyes-on scenarios re-run by the owner after the final code change, since several of the fixes in
   this cycle were only visible to the eye.
4. H1 reported back green from the second machine.
5. H2 decided.
6. Version `1.0.0`.
7. The owner commits and pushes. **Not you.**
