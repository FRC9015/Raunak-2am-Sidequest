// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Shooter {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Alert shooterDisconnectedAlert;

  public Shooter(ShooterIO io) {
    this.io = io;
    shooterDisconnectedAlert = new Alert("Disconnected Shooter", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Update alerts
    shooterDisconnectedAlert.set(!inputs.shooterConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runShooter(int velocity) {
    // Apply setpoints
    io.setShooterVelocity(velocity);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setShooterVelocity(0.0);
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.shooterVelocityRadPerSec;
  }
}
