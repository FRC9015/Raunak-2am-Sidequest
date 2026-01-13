package frc.robot.subsystems.indexer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** TalonFX-based Climb IO implementation (templated). Configure as needed for your robot. */
public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX motor;

  // Control request
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  public IndexerIOTalonFX(int motorId, String canBusName) {
    motor = new TalonFX(motorId, canBusName);

    // Minimal configuration placeholder: users can expand this with their specific configs
    tryUntilOk(
        5,
        () ->
            motor
                .getConfigurator()
                .apply(new com.ctre.phoenix6.configs.TalonFXConfiguration(), 0.25));

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.motorConnected = status.isOK();
    // Phoenix position/velocity units are rotations (i think)
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRPS = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double output) {
    motor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setPositionRotations(double rotations) {
    // This is basic af, can be changed tho, command might also need to be changed
    motor.setControl(voltageRequest.withOutput(0.0));
    motor.setPosition(rotations);
  }

  @Override
  public void setVelocityRPS(double rps) {
    // Template: convert to rotations/sec if required by implementation and set control
    motor.setControl(voltageRequest.withOutput(0.0));
  }
}
