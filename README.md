# **AdvantageKit Extended Template**

### A ready-to-use subsystem architecture with swerve, IO layers, and full simulation support

This repository is an extension of the Base AdvantageKit Template, designed to help FRC teams accelerate development with pre-built subsystem structures, IO abstraction, and simulation-ready components. It includes a fully implemented Swerve drive for a Kraken X60 / Phoenix 6 drivetrain, along with cleanly organized patterns for adding additional mechanisms.


This Template Provides Pre-Built Subsystem Templates, including common subsystem patterns following the **AdvantageKit IO architecture**, such as:

- Example subsystem with IO / IOInputsAutoLogged separation

- RobotContainer structure with clean bindings

- Logging, tuning, and replay already integrated

**You can easily duplicate these patterns to add new mechanisms without rewriting boilerplate.**

## ✔ Swerve Drive Included

- Fully implemented SDS-style Swerve using:

- Kraken X60 motors

- Phoenix 6 Integration

- AdvantageKit auto-logging for telemetry, debugging, and replay

- Realistic simulation for drive kinematics

The drivetrain is ready for **plug-and-play adaptation** to your robot’s parameters.

## ✔ Full Simulation Support

Run robot code without a physical robot using:

- AdvantageScope integration

- Motor and mechanism simulation

- Auto-logged visualization and tuning

This allows testing paths, inputs, and subsystems before the robot is even built.

## 📁 Project Structure
    /src
     ├── main
     │    ├── java
     │    │     ├── frc.robot
     │    │     │     ├── Constants.java
     │    │     │     ├── Robot.java
     │    │     │     ├── RobotContainer.java
     │    │     │     ├── subsystem/
     │    │     │     │     ├── Swerve/
     │    │     │     │     └── ExampleSubsystem/
     │    └── resources
     ├── test



## Key concepts:

- Subsystem = high-level logic

- IO = hardware layer (real or simulated)

- InputsAutoLogged = data pipeline for AK logging

- Replay = use logs for tuning & debugging

## 🧩 Getting Started
  1. Clone the Repo
-     git clone <your-repo-url>

  2. Open in VS Code (WPILib Extension)

  The project already contains all vendor dependencies including _Phoenix 6 and AdvantageKit._

  3. Set Your Robot’s Constants

      - module offsets

      - swerve geometry

      - CAN IDs (Kraken drive + steer)

      - gyro configuration

  4. Run Simulation
-     ./gradlew simulateJava

Open AdvantageScope to visualize state, odometry, and logs.

## 🔧 Customizing for Your Robot

To add a new subsystem:

- Copy the ExampleSubsystem directory

- Rename the folder + classes

- Modify the IO layer to match your hardware

- Add commands & default behaviors in RobotContainer

The IO architecture keeps hardware-specific code isolated and makes unit testing + sim _much cleaner_.

## 📝 Why Use This Template?

- Standardized, competition-ready subsystem architecture

- No wasted time rewriting IO layers every build season

- Built-in tooling for debugging, replay, and control tuning

- Ideal for teams using swerve or building larger codebases

If you’re starting a new robot project or teaching new programmers, this template provides a robust, scalable foundation.

## 📬 Contributions

Feel free to open issues or PRs with improvements, suggestions, or additional subsystem templates.
