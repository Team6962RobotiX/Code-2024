package frc.robot.subsystems.transfer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.util.hardware.NoteDetector;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;
import frc.robot.Constants;
import frc.robot.Constants.AMP.PIVOT;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;
import frc.robot.Presets;

public class TransferInWheels extends SubsystemBase {
  private CANSparkMax motor;
  private NoteDetector detector;
  private State state = State.OFF;
  public static enum State {
    IN,
    OUT,
    OFF,
  }

  public TransferInWheels() {
    if (!ENABLED_SYSTEMS.ENABLE_TRANSFER) return;
    
    motor = new CANSparkMax(CAN.TRANSFER_IN, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, true, IdleMode.kBrake);
    SparkMaxUtil.save(motor);

    detector = new NoteDetector(motor, false);

    Logger.autoLog(this, "hasJustReleaseddNote",    () -> detector.hasJustReleaseddNote());
    Logger.autoLog(this, "hasJustReceivedNote",     () -> detector.hasJustReceivedNote());
  }

  public Command setState(State state) {
    return runOnce(() -> this.state = state);
  }
  
  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_TRANSFER) return;
    if (RobotState.isDisabled()) {
      state = State.OFF;
    }
    switch(state) {
      case IN:
        motor.set(Presets.TRANSFER.IN_POWER);
        break;
      case OUT:
        motor.set(-Presets.TRANSFER.IN_POWER);
        break;
      case OFF:
        motor.set(0);
        break;
    }
  }

  public boolean hasJustReleaseddNote() {
    return detector.hasJustReleaseddNote();
  }

  public boolean hasJustReceivedNote() {
    return detector.hasJustReceivedNote();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
