package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Presets;
import frc.robot.Constants.ENABLED_SYSTEMS;
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
    OFF,
    PICKUP,
    LOAD_AMP,
    LOAD_SHOOTER,
    PLACE_AMP,
    SHOOT,
    INTAKE_OUT
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
    Command command;

    switch(state) {
      case OFF:
        return Commands.sequence(
          amp.setState(Amp.State.OFF),
          intake.setState(Intake.State.OFF),
          shooter.setState(Shooter.State.OFF),
          transfer.setState(Transfer.State.OFF)
        );
      case PICKUP:
        command = Commands.sequence(
          intake.setState(Intake.State.IN),
          transfer.setState(Transfer.State.IN),
          Commands.waitUntil(() -> transfer.hasJustReceivedNote()),
          transfer.setState(Transfer.State.OFF),
          intake.setState(Intake.State.OFF)
        );
        break;
      case LOAD_AMP:
        command = Commands.sequence(
          amp.setState(Amp.State.DOWN),
          amp.setState(Amp.State.IN),
          transfer.setState(Transfer.State.AMP),
          Commands.waitUntil(() -> transfer.hasJustReleaseddNote()),
          transfer.setState(Transfer.State.OFF),
          amp.setState(Amp.State.OFF),
          amp.setState(Amp.State.UP),
          Commands.waitUntil(() -> amp.doneMoving())
        );
        break;
      case LOAD_SHOOTER:
        command = Commands.sequence(
          shooter.setState(Shooter.State.IN),
          // Commands.waitUntil(() -> shooter.doneMoving()),
          transfer.setState(Transfer.State.SHOOTER),
          Commands.waitUntil(() -> transfer.hasJustReleaseddNote()),
          Commands.waitSeconds(0.25),
          shooter.setState(Shooter.State.OFF),
          transfer.setState(Transfer.State.OFF)
        );
        break;
      case PLACE_AMP:
        command = amp.setState(Amp.State.OUT);
        break;
      case SHOOT:
        command = Commands.sequence(
        shooter.setState(Shooter.State.SHOOT)
        );
        break;
      case INTAKE_OUT:
        command = intake.setState(Intake.State.OUT);
        break;
      default:
        command = Commands.run(() -> {});
    }
    return command.finallyDo(() -> setState(State.OFF).schedule());
  }

  @Override
  public void periodic() {
    
  }
}
