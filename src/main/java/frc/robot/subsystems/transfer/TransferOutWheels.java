package frc.robot.subsystems.transfer;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.SparkMaxUtil;

public class TransferOutWheels extends SubsystemBase {
  private CANSparkMax motor;
  private State state = State.OFF;
  public static enum State {
    AMP,
    SHOOTER,
    AMP_OUT,
    SHOOTER_OUT,
    OFF,
  }

  public TransferOutWheels() {    
    motor = new CANSparkMax(CAN.TRANSFER_OUT, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, false, CANSparkMax.IdleMode.kBrake);
    SparkMaxUtil.configureCANStatusFrames(motor, false, false);
    SparkMaxUtil.save(motor);
  }

  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }
  
  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_TRANSFER) return;
    if (RobotState.isDisabled()) {
      state = State.OFF;
    }
    switch(state) {
      case AMP:
      case SHOOTER_OUT:
        motor.set(-Preferences.TRANSFER.OUT_POWER_TOP);
        break;
      case SHOOTER:
      case AMP_OUT:
        motor.set(Preferences.TRANSFER.OUT_POWER_TOP);
        break;
      case OFF:
        motor.set(0);
        break;

    }
    
    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.TRANSFER) motor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
