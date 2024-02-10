package frc.robot.subsystems.transfer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;



public class TransferWheels extends SubsystemBase {
  private CANSparkMax transferIn;
  private CANSparkMax transferOut;
  private TransferState state = TransferState.OFF;
 
  public static enum TransferState {
    AMP,
    SHOOTER,
    OFF,
  }

  public TransferWheels() {
    if (!ENABLED_SYSTEMS.ENABLE_TRANSFER) return;
    
    transferIn = new CANSparkMax(CAN.TRANSFER_IN, MotorType.kBrushless);
    transferOut = new CANSparkMax(CAN.TRANSFER_OUT, MotorType.kBrushless);
  }

  public void setState(TransferState newState) {
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
