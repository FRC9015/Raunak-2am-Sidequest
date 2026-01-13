package frc.robot.subsystems.indexer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;
import org.littletonrobotics.junction.Logger;

public class Indexer {
  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private final PIDController positionController = new PIDController(kP, kI, kD);
  private double positionSetpoint = 0.0;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb/Motor", inputs);

    // Closed-loop position control (simple PIDs via WPILib controller)
    double output = positionController.calculate(inputs.positionRotations, positionSetpoint);
    // If setpoint is zero and small, allow open-loop stop
    if (Math.abs(positionSetpoint) < 1e-6) {
      // do nothing here, user may call stop() which will set open loop to 0
    } else {
      io.setOpenLoop(output);
    }
  }

  /** Runs the indexer motor open loop in [-1, 1]. */
  public void runOpenLoop(double output) {
    io.setOpenLoop(output);
  }

  /** Move to a position in rotations (template). */
  public void moveToPosition(double rotations) {
    positionSetpoint = rotations;
    positionController.setSetpoint(rotations);
  }

  /** Stop the indexer immediately. */
  public void stop() {
    positionSetpoint = 0.0;
    positionController.reset();
    io.setOpenLoop(0.0);
  }

  /** Returns a simple command that moves to the given rotation setpoint. */
  public Command moveToPositionCommand(double rotations) {
    return Commands.runOnce(() -> moveToPosition(rotations));
  }
}
