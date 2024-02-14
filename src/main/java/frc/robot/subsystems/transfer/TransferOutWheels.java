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
import frc.robot.subsystems.notes.NoteDetector;
import frc.robot.util.ConfigUtils;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;
import frc.robot.Constants;
import frc.robot.Constants.AMP.PIVOT;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;
import frc.robot.Constants.TRANSFER;
import frc.robot.Presets;

public class TransferOutWheels extends SubsystemBase {
  private CANSparkMax motor;
  private State state = State.OFF;
  public static enum State {
    AMP,
    SHOOTER,
    OFF,
  }

  public TransferOutWheels() {
    if (!ENABLED_SYSTEMS.ENABLE_TRANSFER) return;
    
    motor = new CANSparkMax(CAN.TRANSFER_OUT, MotorType.kBrushless);

    ConfigUtils.configure(List.of(
      () -> motor.restoreFactoryDefaults(),
      () -> { motor.setInverted(true); return true; },
      () -> motor.setIdleMode(IdleMode.kBrake),
      () -> motor.enableVoltageCompensation(12.0),
      () -> motor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, PIVOT.PROFILE.CURRENT_LIMIT),
      () -> motor.setClosedLoopRampRate(PIVOT.PROFILE.RAMP_RATE),
      () -> motor.burnFlash()
    ));

    String logPath = "transfer-out-wheels/";
    Logger.autoLog(logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> motor.getMotorTemperature());
    
    StatusChecks.addCheck("Transfer Out Motor", () -> motor.getFaults() == 0);
  }

  public Command setState(State state) {
    return runOnce(() -> this.state = state);
  }
  
  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_TRANSFER) return;
    switch(state) {
      case AMP:
        motor.set(Presets.TRANSFER.OUT_POWER);
      case SHOOTER:
        motor.set(-Presets.TRANSFER.OUT_POWER);
      case OFF:
        motor.set(0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
