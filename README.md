# Glitch-2.0-2026-chassisbot

> **FRC Team 8727 — Glitch 2.0**  
> Season: **2026 REBUILT™** presented by Haas  
> Codebase: WPILib (GradleRIO 2026.2.1) · Java 17  
> Primary robot: Swerve drive (CTRE TalonFX / Phoenix 6)

---

## ⚠️ Repository name note

This repo is named `Glitch-2.0-2026-chassisbot` for historical reasons. **It is not a testbed.** It is the team's actual 2026 competition robot — the same robot that went 32-17-0, won the UNC Asheville district event, and earned an Autonomous Award.

The "chassisbot" label was a working name that stuck. The context repo at `Glitch-2.0-Agent-Context/` previously questioned whether this repo was an offseason testbed; that question has been resolved. **The code here is the authoritative record of the 2026 competition robot.**

---

## Repository structure

```
src/main/java/frc/robot/
├── Commands/              — ShootCommand (4-mode shooter)
├── controller/            — ProjectileSolver, swerve controls, bindings
├── Drivetrain/            — CTRESwerveDrivetrain, Phoenix 6 config
├── Subsystems/            — ShooterRoller, Indexer, IntakeRoller, LEDSubsystem
├── Autos.java             — PathPlanner autonomous routines (23 paths)
├── Robot.java             — Main loop, field geometry, target constants
└── Vision.java            — PhotonVision camera config

src/main/deploy/
├── elastic-layout.json    — Elastic dashboard (4 tabs including Diagnostic)
└── pathplanner/           — PathPlanner navgrid, settings, 23 path files

src/test/                  — 26 unit tests (JUnit 5)

.github/workflows/         — CI pipeline (build + test + JSON validation)
```

---

## Current sprint: THOR West 2026

This branch (`THOR`) is being prepared for **THOR West**, an FRC off-season competition running the REBUILT game:

| Detail | Value |
|--------|-------|
| Event | **THOR West** off-season |
| Date | **Saturday, October 24, 2026** |
| Venue | R-S Central High School, 641 US-221, Rutherfordton, NC 28139 |
| Host | FRC 5727 Omegabytes |
| Game | REBUILT™ presented by Haas |

The work on this branch is focused on making the robot field-ready for this event. The primary blockers are pose XY estimation (Issue 1) and verifying the recently-corrected shooter target coordinate frame (Issue 2).

---

## Build and test

```bash
./gradlew build        # Compile everything (16 main classes)
./gradlew test         # Run all 26 unit tests (robot + GlitchLib)
```

CI runs automatically on every push and pull request via `.github/workflows/ci.yml`.

---

## Key known issues

These are tracked in detail in `Glitch-2.0-Agent-Context/seasons/2026-rebuilt/robot/known_issues.md`. The highest-priority items:

| # | Issue | Status |
|---|-------|--------|
| 1 | **Pose XY estimation** — robot's field position doesn't update reliably | ⛔ blocker |
| 2 | ~~Shooter target coordinate frame — was in the wrong coordinate system~~ | **fixed Oct 5** — unverified on-robot |
| 3 | Flywheel gearing ratio — `0.5` vs `24/15` needs resolution | open |
| 4 | Intake + Indexer PID untuned — `pid(0,0,0)` | open |
| 5 | PathPlanner field origin — paths have Y=5.97 (impossible in center-origin) | open |

---

## Context repository

Detailed team and season context lives in the sibling repository:
```
Glitch-2.0-Agent-Context/
```
Start with `START-HERE.md`, then read the mandatory absorb order. This README is a high-level summary only; the context repo is the source of truth for game rules, unit/frame conventions, engineering practices, and blocking constraints.

---

*Maintained by Team 8727. Oct 5, 2026 — 19 days until THOR West.*