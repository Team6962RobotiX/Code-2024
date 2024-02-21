package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;

// This class is a subsystem that controls the state of the robot. It is used to coordinate the actions of the intake, shooter, transfer, and amp subsystems.

public class RobotStateController extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private Amp amp;
  private Intake intake;
  private Shooter shooter;
  private Transfer transfer;

  public enum State {
    INTAKE,
    INTAKE_OUT,
    PREPARE_AMP,
    PLACE_AMP,
    LEAVE_AMP,
    PREPARE_SPEAKER,
    SHOOT_SPEAKER,
    PREPARE_SOURCE,
    INTAKE_SOURCE,
    PREPARE_TRAP,
    SHOOT_TRAP
  }

  public RobotStateController(Amp amp, SwerveDrive swerveDrive, Intake intake, Shooter shooter, Transfer transfer) {
    this.swerveDrive = swerveDrive;
    this.amp = amp;
    this.intake = intake;
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
          intake.setState(Intake.State.IN),
          transfer.setState(Transfer.State.IN)
        ).until(() -> transfer.hasNote());
      case INTAKE_OUT:
        return Commands.parallel(
          intake.setState(Intake.State.OUT),
          transfer.setState(Transfer.State.OUT)
        ).until(() -> !intake.hasNote());
      case PREPARE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN),
          amp.setState(Amp.State.IN).alongWith(
            transfer.setState(Transfer.State.AMP)
          ).until(() -> !transfer.hasNote()),
          amp.setState(Amp.State.UP)
        );
      case PLACE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.UP),
          amp.setState(Amp.State.OUT).until(() -> !amp.hasNote())
        );
      case LEAVE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN)
        );
      case PREPARE_SPEAKER:
        return Commands.parallel(
          shooter.setState(Shooter.State.IN),
          transfer.setState(Transfer.State.SHOOTER)
        ).until(() -> !transfer.hasNote());
      case SHOOT_SPEAKER:
        return Commands.sequence(
          shooter.setState(Shooter.State.SHOOT)
        );
      default:
        return Commands.run(() -> {});
    }
  }

  @Override
  public void periodic() {
    
  }
}
