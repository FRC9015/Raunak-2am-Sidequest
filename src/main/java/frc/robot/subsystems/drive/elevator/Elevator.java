package frc.robot.subsystems.drive.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.subsystems.drive.elevator.ElevatorIO.ElevatorIOInputs.ElevatorState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;


public class Elevator {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private  Alert elevatorDisconnectedAlert;
  private  Alert encoderDisconnectedAlert;

  private  PIDController pidController;
  private  ElevatorFeedforward elevatorFeedforward;

  private double kP = 5;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kS = 0;
  private double kG = 1;
  private double kV = 0;
  private double kA = 0.0;
  private double offset = 0.0;

  private double acceptableTolerance = 0.1; // in meters
  private double lastPosition = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;
    elevatorDisconnectedAlert = new Alert("Disconnected Shooter", AlertType.kError);
    this.pidController = new PIDController(kP, kI, kD);
    this.elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    this.pidController.setTolerance(acceptableTolerance);
  }

  @Override
  public void periodic() {
      io.updateInputs(inputs);
      Logger.processInputs("Elevator", inputs);
      encoderDisconnectedAlert.set(!inputs.elevatorEncoderConnected);

  }

  public void setElevatorPosition(ElevatorState state) {
    double targetPosition = state.getEncoderPosition();
    pidController.setSetpoint(targetPosition);  

    double output = pidController.calculate(inputs.elevatorPosition)+ elevatorFeedforward.calculate(inputs.elevatorPosition);

    inputs.setpoint = targetPosition;

    io.setElevatorPosition(state.getEncoderPosition());
    io.updateInputs(inputs);
    lastPosition = inputs.elevatorPosition;

    
  }

  public Command executePreset(ElevatorIOInputs.ElevatorState state) {
    Logger.recordOutput("Elevator/State", state);
    Logger.recordOutput("Elevator/CurrentPosition", inputs.elevatorPosition + offset);
    return Commands.run(() -> this.setElevatorPosition(state));
  }

  public Command executePreset(Supplier<ElevatorState> supplier) {
    Logger.recordOutput("state: ", supplier.get());
    return Commands.run(() -> this.setElevatorPosition(supplier.get()));
  }

  public void setBrakeMode() {
    io.setBrakeMode();
  }

  public void setCoastMode() {
    io.setCoastMode();
  }

  /**
   * Checks if the elevator is at the set position.
   *
   * @return True if the elevator is at the set position, false otherwise.
   */
  public Boolean elevatorAtPosition() {
    return inputs.elevatorAtSetpoint;
  }

  /**
   * Zeros the elevator position.
   *
   * @return A command that zeros the elevator.
   */
  public Command zeroTheElevator() {
    return Commands.run(() -> io.zeroElevator());
  }

  public Boolean getElevatorLimitSwitch() {
    return inputs.zeroSwitchTriggered;
  }

  /**
   * Gets the current state of the elevator.
   *
   * @return The current state of the elevator.
   */
  public ElevatorState getElevatorState() {
    return inputs.elevatorState;
  }

  /** Toggles toggle. */
  public Command toggle() {
    return Commands.runOnce(io::switchToggle);
  }

  public boolean getToggle() {
    return inputs.toggle;
  }

}

