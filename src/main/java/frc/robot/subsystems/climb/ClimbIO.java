package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** IO abstraction for the climb mechanism. Implementations provide hardware interfaces. */
public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    /* Change values later as needed for your robot */
    public boolean motorConnected = false;
    public double positionRotations = 0.0;
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates observable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Open-loop control in [-1, 1] or a voltage request depending on implementation. */
  public default void setOpenLoop(double output) {}

  /** Closed-loop position command (rotations). */
  public default void setPositionRotations(double rotations) {}

  /** Closed-loop velocity command (rotations per second). */
  public default void setVelocityRPS(double rps) {}
}
