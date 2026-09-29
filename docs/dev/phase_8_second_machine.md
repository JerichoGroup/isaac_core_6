# Phase 8 Stage 2.5 — the second machine

This is the only test of the **install path** and of **Isaac Sim 6.1 compatibility**. Everything else in
Phase 8 was verified on one machine, against 6.0.1-rc.7, from a working tree that was never installed from
scratch. A missing file, an undeclared dependency, or a 6.1 behaviour change would all be invisible here and
obvious there.

Run this on the 6.1.0-rc.26 box.

---

## Before you start

**1. Push the branch you want tested.** The script clones from the remote, so it verifies whatever is on that
branch — not your working tree. As of writing, the V3 work is on `dev` with uncommitted changes, so pushing
is the first step or the run will faithfully verify the wrong code. The script checks for a few files that
only exist in the V3 work and stops if they are absent, so a stale branch fails loudly rather than passing
quietly.

**2. Keep the GPU free.** A Cesium stage needs roughly 3 GiB. A resident language model or other GPU workload
starves the renderer *silently* — the simulator composes and then stops responding with nothing saying why.
This cost hours to diagnose on the first machine.

```bash
nvidia-smi --query-gpu=memory.free --format=csv,noheader     # want > 4000 MiB
```

**3. Have a reachable Cesium tileset**, or expect no terrain. Without one the simulator still runs and the
camera still moves; you just see no ground, and the log carries `curl: Couldn't connect to server`.

---

## Run it

```bash
cd ~/clones/isaac_core_6      # any existing clone, just to get the script
git pull
./scripts/fresh_clone_check.sh git@github.com:JerichoGroup/isaac_core_6.git dev
```

It clones into a fresh temporary directory rather than reusing your checkout, which is the point: an existing
tree can satisfy a dependency that a new machine would not have.

Roughly 5–10 minutes, most of it `setup.sh` and the first flight. Nothing it does is destructive: it clones
to `/tmp`, installs with `pip install --user`, and deletes nothing.

### What it does, in order

1. Records the machine: OS, kernel, GPU, driver, free GPU memory, Python, Isaac version, ROS distro.
2. Clones the branch and **confirms it is the code under test**, stopping if not.
3. Runs `./scripts/setup.sh` — the documented installer. **This is the interesting part.**
4. Runs `isaac-core doctor`.
5. Runs the unit suite.
6. Runs `isaac-core run --dry-run`.
7. Launches headless for 45 s, capped at 90, and checks the control plane answers.
8. Prints where the report and the simulator log are.

---

## What to report back

Send the **report file** the script prints at the end. That is the primary artefact and it carries everything
below. Then, in your own words:

**The five things that matter most**

1. **Did `setup.sh` finish, exit 0?** If not, the exact step that failed and its output. This is the whole
   reason for the exercise — the installer has never run on a machine that did not already have everything.
2. **Did `doctor` report all checks passing?** Any `[FAIL]` line verbatim, especially the Isaac version line,
   since 6.1 has never been checked.
3. **Did the unit suite pass?** The summary line. A failure here on 6.1 but not 6.0.1 is a compatibility
   finding, which is exactly what we are looking for.
4. **Did the control plane answer** in the first-flight step, `ANSWERING` or `NOT ANSWERING`?
5. **How many error lines** the simulator log reported, and any that mention `isaac_core` rather than `omni`,
   `carb` or `cesium` — ours are the ones we can act on.

**Also worth a line**

- The Isaac version the script recorded, so the result is attributable.
- Anything that took far longer than it seems it should have.
- Anything in `setup.sh`'s output that looked like a warning you would not want a new user to see.

**If you have another three minutes**, the system tests are the strongest signal available and are skipped by
default. The script prints the command; it is:

```bash
cd <the temp clone the script names> && python3 -m pytest tests/system -q --system
```

Report the summary line. Expect 27 passed. A swarm skip with a message about the simulation thread not
servicing calls is the known two-vehicle sensitivity, not a 6.1 problem — but say so if you see it, because
seeing it on a second machine would tell us something the first machine could not.

---

## What a failure here means

Nothing found on this machine invalidates the work done on the first one; it tells us which parts were
machine-specific. Three outcomes are all useful:

- **Everything passes.** The install path works from scratch and 6.1 is compatible. That is the result that
  clears 1.0.0.
- **`setup.sh` fails.** A real bug in the one path no amount of testing on a configured machine can exercise.
- **Something passes on 6.0.1 and fails on 6.1.** A compatibility finding, and better discovered now than by
  whoever upgrades next.

The one outcome to be careful about is a **green report from the wrong branch**, which is why the script
checks and stops. If it stops for that reason, push and rerun rather than overriding it.
