package frc.robot.subsystems.transfer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.util.ConfigUtils;
import frc.robot.util.NoteDetector;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;
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

    ConfigUtils.configure(List.of(
      () -> motor.restoreFactoryDefaults(),
      () -> { motor.setInverted(true); return true; },
      () -> motor.setIdleMode(IdleMode.kBrake),
      () -> motor.enableVoltageCompensation(12.0),
      () -> motor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, PIVOT.PROFILE.CURRENT_LIMIT),
      () -> motor.setClosedLoopRampRate(PIVOT.PROFILE.RAMP_RATE),
      () -> motor.setOpenLoopRampRate(NEO.SAFE_RAMP_RATE),
      () -> motor.burnFlash()
    ));

    detector = new NoteDetector(motor);

    String logPath = "transfer-in-wheels/";
    Logger.autoLog(logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> motor.getMotorTemperature());
    Logger.autoLog(logPath + "hasJustReleaseddNote",    () -> detector.hasJustReleaseddNote());
    Logger.autoLog(logPath + "hasJustReceivedNote",     () -> detector.hasJustReceivedNote());

    
    StatusChecks.addCheck("Transfer In Motor", () -> motor.getFaults() == 0);
  }

  public Command setState(State state) {
    return runOnce(() -> this.state = state);
  }
  
  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_TRANSFER) return;
    switch(state) {
      case IN:
        motor.set(Presets.TRANSFER.IN_POWER);
      case OUT:
        motor.set(-Presets.TRANSFER.IN_POWER);
      case OFF:
        motor.set(0);
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
