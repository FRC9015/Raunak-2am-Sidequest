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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ShooterIOTalonFX implements ShooterIO {

  // Hardware objects
  private final TalonFX flywheelTalon;

  // Voltage control requests
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> shooterVelocity;
  private final StatusSignal<Voltage> shooterAppliedVolts;
  private final StatusSignal<Current> shooterCurrent;

  // Connection debouncers
  private final Debouncer shooterConnectedDebounce = new Debouncer(0.5);

  public ShooterIOTalonFX() {
    flywheelTalon = new TalonFX(Constants.Shooter.ShooterMotorID);
    flywheelTalon.getConfigurator().apply(Constants.Shooter.ShooterConfiguration);

    // Create drive status signals
    shooterVelocity = flywheelTalon.getVelocity();
    shooterAppliedVolts = flywheelTalon.getMotorVoltage();
    shooterCurrent = flywheelTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, shooterVelocity, shooterAppliedVolts, shooterCurrent);
    ParentDevice.optimizeBusUtilizationForAll(flywheelTalon);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Refresh all signals
    var shooterStatus =
        BaseStatusSignal.refreshAll(shooterVelocity, shooterAppliedVolts, shooterCurrent);

    // Update drive inputs
    inputs.shooterConnected = shooterConnectedDebounce.calculate(shooterStatus.isOK());
    inputs.shooterVelocityRadPerSec = Units.rotationsToRadians(shooterVelocity.getValueAsDouble());
    inputs.shooterAppliedVolts = shooterAppliedVolts.getValueAsDouble();
    inputs.shooterCurrentAmps = shooterCurrent.getValueAsDouble();
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    flywheelTalon.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
  }
}
