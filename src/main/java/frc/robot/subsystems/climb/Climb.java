package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbIOInputs inputs = new ClimbIOInputs();

    // replace with tuned values, if this is still 0 ts won't work
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final PIDController positionController = new PIDController(kP, kI, kD);
    private double positionSetpoint = 0.0;

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
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

    /** Runs the climber motor open loop in [-1, 1]. */
    public void runOpenLoop(double output) {
        io.setOpenLoop(output);
    }

    /** Move to a position in rotations (template). */
    public void moveToPosition(double rotations) {
        positionSetpoint = rotations;
        positionController.setSetpoint(rotations);
    }

    /** Stop the climber immediately. */
    public void stop() {
        positionSetpoint = 0.0;
        positionController.reset();
        io.setOpenLoop(0.0);
    }

    /** Returns a simple command that moves to the given rotation setpoint. */
    public Command moveToPositionCommand(double rotations) {
        return runOnce(() -> moveToPosition(rotations));
    }
}