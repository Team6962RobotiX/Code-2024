package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;

// This class is a subsystem that controls the state of the robot. It is used to coordinate the actions of the intake, shooter, transfer, and amp subsystems.

public class RobotStateController extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private Amp amp;
  private Shooter shooter;
  private Transfer transfer;

  public enum State {
    INTAKE,
    INTAKE_OUT,
    PREPARE_AMP,
    PLACE_AMP,
    LEAVE_AMP,
    PREPARE_SPEAKER,
    AIM_SPEAKER,
    SHOOT_SPEAKER,
    PREPARE_SOURCE,
    INTAKE_SOURCE,
    PREPARE_TRAP,
    SHOOT_TRAP
  }

  public RobotStateController(Amp amp, SwerveDrive swerveDrive, Shooter shooter, Transfer transfer) {
    this.swerveDrive = swerveDrive;
    this.amp = amp;
    this.shooter = shooter;
    this.transfer = transfer;
  }

  /**
   * Sets the state of the robot
   * @param state
   * @return
   */

  public Command setState(State state) {
    switch(state) {
      case INTAKE:
        return Commands.parallel(
          transfer.setState(Transfer.State.IN)
        ).until(() -> transfer.hasNote() || Robot.isSimulation());
      case INTAKE_OUT:
        return Commands.parallel(
          transfer.setState(Transfer.State.OUT)
        ).until(() -> !transfer.hasNote() || Robot.isSimulation());
      case PREPARE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN),
          amp.setState(Amp.State.IN).alongWith(
            transfer.setState(Transfer.State.AMP)
          ).until(() -> !transfer.hasNote() || Robot.isSimulation()),
          amp.setState(Amp.State.UP)
        );
      case PLACE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.UP),
          amp.setState(Amp.State.OUT).until(() -> !amp.hasNote() || Robot.isSimulation())
        );
      case LEAVE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN)
        );
      case PREPARE_SPEAKER:
        return Commands.parallel(
          shooter.setState(Shooter.State.IN),
          transfer.setState(Transfer.State.SHOOTER)
        ).until(() -> !transfer.hasNote() || Robot.isSimulation());
      case AIM_SPEAKER:
        return shooter.setState(Shooter.State.AIM);
      case SHOOT_SPEAKER:
        return shooter.setState(Shooter.State.SHOOT);
      default:
        return Commands.run(() -> {});
    }
  }

  @Override
  public void periodic() {
    
  }
}
