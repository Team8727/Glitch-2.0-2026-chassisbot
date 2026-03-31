# Glitch 2.0 Agents Documentation

This document serves to provide instructions and guidelines for agents to write, alter, suggest, and test code within this Glitch 2.0 project. All agents should adhere to the following sections to ensure consistency and quality in contributions, unless the user specifies otherwise (by explicitly requesting a different approach).

## Project overview

This project is the repository for FRC (FIRST Robotics Competition) Team 8727 (Glitch 2.0)'s 2026 codebase. It is built using the WPILib framework and is designed to run on the FRC robots. The codebase includes subsystems, commands, and other files and configurations necessary for robot operation in simulation and real time during the season, during competitions, and during the off-season.
- Swerve is powered by CTRE Phoenix 6 (`frc.robot.Drivetrain.CTRESwerveDrivetrain`, `TunerConstants`), using CAN bus name "Swerve Drive" and CTRE swerve geometry/limits defined in `kMax*` constants.
- Autos use PathPlanner (`Autos`, `deploy/pathplanner` paths). Named commands are registered in `Autos.registerNamedCommands` (e.g., `spinRollers`, `shoot`).
- Vision uses PhotonVision via `frc.robot.Vision` with four camera configs; vision is attached to drivetrain through `CTRESwerveDrivetrain.setVision` and drained in `periodic`.
- Teleop control is command-based with default swerve command set in `controller/CTReSwerveControls`; shooter/intake subsystems set default commands to hold zero output.
- In-repo GlitchLib (`GlitchLib/Glitch/Lib/main`) provides controllers, logging (`NetworkTableLogger`), and mechanisms (Pivot, Roller) used across subsystems.

## Build and test commands
To build and test the project, agents should use the following commands:
- To build the project: `./gradlew build`
- To run tests: `./gradlew :test`
- On Windows shells, prefer `gradlew.bat` equivalents. For WPILib sim, use `./gradlew simulateJava`.

## Guidelines for contributing

Please read the file GLITCHDOCS.md for advice before suggesting advice. All contributions must pass all tests, the project should be able to build successfully, and agents should adhere to the project's coding standards (unless the user explicitly requests a different approach). Agents should ensure that their code is well-documented through commenting in the code, logical formatting, and explanation to the user and follows best practices for readability and maintainability.
- Preserve PathPlanner integration: update `Autos.registerNamedCommands` when adding in-path behaviors and keep auto names consistent with `deploy/pathplanner` files.
- Keep command-based patterns: default swerve command is set in `CTReSwerveControls` via `drivetrain.applyRequest`; subsystems under `Subsystems/` use `setDefaultCommand` to idle safely.
- Use `NetworkTableLogger` (or WPILib dashboards) for telemetry in `periodic` loops; avoid one-off prints.
- Maintain swerve/constants alignment in `TunerConstants` when changing geometry, CAN IDs, or speed limits.

## Code style guidelines
- Follow command-based organization: commands in `Commands/`, bindings in `controller/`, subsystems in `Subsystems/` with minimal cross-coupling.
- Prefer lambda-based `run`/`sequence`/`parallel` helpers (see `ShootCommand`, `Driver1DefaultBindings`) for concise commands.
- Keep unit-safe math (WPILib `Units`, CTRE `Units`) and avoid magic numbers for angles/distances where possible.
- When adding logging, reuse existing `NetworkTableLogger` instance per class to minimize topic churn.

## Testing instructions
- Run `./gradlew :test` before changes; add tests under `src/test/java` if new logic is added.
- Use `./gradlew simulateJava` for WPILib sim; ensure swerve and vision still initialize (vision provider selection is `Robot.isSimulation()` aware).
- Validate autos via SmartDashboard chooser (populated in `Autos.setupAutoChooser`) and ensure new named commands are registered before loading paths.

## Security considerations
- Do not commit secrets or external credentials; keep team numbers, CAN IDs, and camera names accurate.
- Confirm firmware/vendor deps (CTRE/REV) are up to date before hardware deploys; avoid hardcoding field IPs or Wi-Fi credentials.
