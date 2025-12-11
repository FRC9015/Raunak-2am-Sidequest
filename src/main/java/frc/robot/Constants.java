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

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ElevatorConstants {
    public static final double ELEVATOR_MAGNET_OFFSET = -0.13;
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(150).withMotionMagicCruiseVelocity(50);
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(0)
            .withKI(0)
            .withKD(0.0)
            .withKG(0.0)
            .withKA(0)
            .withKS(0)
            .withKV(0);
    public static final FeedbackConfigs FEEDBACK_CONFIGS =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(0)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

    public static final double maxHeight = 8.25;
    public static final double minHeight = -0.04;
  }
}
