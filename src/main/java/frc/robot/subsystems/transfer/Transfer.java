package frc.robot.subsystems.transfer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.util.ConfigUtils;
import frc.robot.Constants;
import frc.robot.Constants.AMP.PIVOT;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;



public class Transfer extends SubsystemBase {
  private CANSparkMax transferIn;
  private CANSparkMax transferOut;
  private State state = State.OFF;
 
  public static enum State {
    AMP,
    SHOOTER,
    OFF,
  }

  public Transfer() {
    if (!ENABLED_SYSTEMS.ENABLE_TRANSFER) return;
    
    transferIn = new CANSparkMax(CAN.TRANSFER_IN, MotorType.kBrushless);
    transferOut = new CANSparkMax(CAN.TRANSFER_OUT, MotorType.kBrushless);

    ConfigUtils.configure(List.of(
      () -> transferIn.restoreFactoryDefaults(),
      () -> { transferIn.setInverted(true); return true; },
      () -> transferIn.setIdleMode(IdleMode.kBrake),
      () -> transferIn.enableVoltageCompensation(12.0),
      () -> transferIn.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, PIVOT.PROFILE.CURRENT_LIMIT),
      () -> transferIn.setClosedLoopRampRate(PIVOT.PROFILE.RAMP_RATE),
      () -> transferIn.burnFlash(),

      () -> transferOut.restoreFactoryDefaults(),
      () -> { transferOut.setInverted(true); return true; },
      () -> transferOut.setIdleMode(IdleMode.kBrake),
      () -> transferOut.enableVoltageCompensation(12.0),
      () -> transferOut.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, PIVOT.PROFILE.CURRENT_LIMIT),
      () -> transferOut.setClosedLoopRampRate(PIVOT.PROFILE.RAMP_RATE),
      () -> transferOut.burnFlash()
    ));
  }

  public void setState(State newState) {
    state = newState;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_TRANSFER) return;

    switch(state) {
      case OFF:
        transferIn.set(0);
        transferOut.set(0);
        break;
      case AMP:
        transferIn.set(-0.25);
        transferOut.set(0.25);
        break;
      case SHOOTER:
        transferIn.set(-0.25);
        transferOut.set(-0.25);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
