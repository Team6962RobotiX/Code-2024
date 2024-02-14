package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Presets;
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

  private State state = State.OFF;
  public enum State {
    OFF,
    PICKUP,
    LOAD_AMP,
    LOAD_SHOOTER,
    PLACE_AMP,
    SHOOT
  }

  public RobotStateController(Amp amp, SwerveDrive swerveDrive, Intake intake, Shooter shooter, Transfer transfer) {
    this.swerveDrive = swerveDrive;
    this.amp = amp;
    this.intake = intake;
    this.shooter = shooter;
    this.transfer = transfer;
    setState(state).schedule();
  }

  /**
   * Sets the state of the robot
   * @param state
   * @return
   */

  public Command setState(State state) {
    switch(state) {
      case OFF:
        return Commands.sequence(
          amp.setState(Amp.State.OFF),
          intake.setState(Intake.State.OFF),
          shooter.setState(Shooter.State.OFF),
          transfer.setState(Transfer.State.OFF)
        );
      case PICKUP:
        return Commands.sequence(
          intake.setState(Intake.State.IN),
          transfer.setState(Transfer.State.IN),
          Commands.waitSeconds(1.0),
          intake.setState(Intake.State.OFF),
          transfer.setState(Transfer.State.OFF)
        );
      case LOAD_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.IN),
          transfer.setState(Transfer.State.AMP),
          Commands.waitUntil(() -> amp.hasNote()),
          amp.setState(Amp.State.OFF),
          transfer.setState(Transfer.State.OFF)
        );
      case LOAD_SHOOTER:
        return Commands.sequence(
          shooter.setState(Shooter.State.IN),
          Commands.waitUntil(() -> shooter.doneMoving()),
          transfer.setState(Transfer.State.SHOOTER),
          shooter.setState(Shooter.State.IN),
          Commands.waitUntil(() -> shooter.hasNote()),
          shooter.setState(Shooter.State.OFF),
          transfer.setState(Transfer.State.OFF)
        );
      case PLACE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.OUT),
          transfer.setState(Transfer.State.OFF)
        );
      case SHOOT:
        return Commands.sequence(
          shooter.setState(Shooter.State.SHOOT)
        );
    }
    return null;
  }

  @Override
  public void periodic() {
    
  }
}
