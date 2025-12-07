package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

  // Combined moment of inertia for 3 AndyMark 6" HiGrip wheels (kg*m^2)
  private static final double kFlywheelInertia = 0.001515;

  // Hood drag constants (tune to match real flywheel)
  private static final double kViscous = 0.0015;
  private static final double kAir = 0.000002;

  // Motor and plant
  private final DCMotor gearbox = DCMotor.getFalcon500(1);

  private final FlywheelSim shooterSim =
      new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox, 1.0, kFlywheelInertia), gearbox);

  // Commanded voltage from velocity controller
  private double commandedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Current flywheel speed
    double omega = shooterSim.getAngularVelocityRadPerSec();

    // Compute hood drag torque
    double hoodDragTorque = kViscous * omega + kAir * omega * omega;

    // Convert drag torque to motor voltage
    double torqueAtMotor = hoodDragTorque / shooterSim.getGearing();
    double currentRequired = torqueAtMotor / gearbox.KtNMPerAmp;
    double backdriveVolts = currentRequired * gearbox.rOhms;

    // Net voltage applied
    double netVolts = commandedVolts - backdriveVolts;
    netVolts = Math.max(-12.0, Math.min(12.0, netVolts));

    // Step simulation
    shooterSim.setInputVoltage(netVolts);
    shooterSim.update(0.02);

    // Populate ShooterIOInputs
    inputs.shooterConnected = true;
    inputs.shooterVelocityRadPerSec = shooterSim.getAngularVelocityRadPerSec();
    inputs.shooterAppliedVolts = commandedVolts;
    inputs.shooterCurrentAmps = shooterSim.getCurrentDrawAmps();
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    // Simple proportional controller for sim
    double error = velocityRadPerSec - shooterSim.getAngularVelocityRadPerSec();
    commandedVolts = error * 0.02; // tune gain as needed
    commandedVolts = Math.max(-12.0, Math.min(12.0, commandedVolts));
  }
}
