package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;



public class Transfer extends SubsystemBase {
  private TransferInWheels transferIn;
  private TransferOutWheels transferOut;
 
  public static enum State {
    IN,
    OUT,
    AMP,
    SHOOTER,
  }

  public Transfer() {    
    transferIn = new TransferInWheels();
    transferOut = new TransferOutWheels();
  }

  public TransferInWheels getInWheels() {
    return transferIn;
  }

  public TransferOutWheels getOutWheels() {
    return transferOut;
  }

  public Command setState(State state) {
    switch(state) {
      case IN:
        return Commands.parallel( 
          transferIn.setState(TransferInWheels.State.IN),
          transferOut.setState(TransferOutWheels.State.OFF)
        );
      case OUT:
        return Commands.parallel( 
          transferIn.setState(TransferInWheels.State.OUT),
          transferOut.setState(TransferOutWheels.State.OFF)
        );
      case AMP:
        return Commands.parallel( 
          transferIn.setState(TransferInWheels.State.IN),
          transferOut.setState(TransferOutWheels.State.AMP)
        );
      case SHOOTER:
        return Commands.parallel( 
          transferIn.setState(TransferInWheels.State.IN),
          transferOut.setState(TransferOutWheels.State.SHOOTER)
        );
    }
    return null;
  }
  
  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_TRANSFER) return;
  }

  public boolean hasNote() {
    return transferIn.hasNote();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
