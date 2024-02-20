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
      case OFF:
        return Commands.sequence(
          amp.setState(Amp.State.OFF),
          intake.setState(Intake.State.OFF),
          shooter.setState(Shooter.State.OFF),
          transfer.setState(Transfer.State.OFF)
        );
      case INTAKE:
        return Commands.sequence(
          intake.setState(Intake.State.IN),
          transfer.setState(Transfer.State.IN),
          Commands.waitUntil(() -> transfer.hasNote())
        ).finallyDo(() -> Commands.sequence(
          intake.setState(Intake.State.OFF),
          transfer.setState(Transfer.State.OFF)
        ).schedule());
      case INTAKE_OUT:
        return Commands.sequence(
          intake.setState(Intake.State.OUT),
          transfer.setState(Transfer.State.OUT),
          Commands.waitUntil(() -> !intake.hasNote())
        ).finallyDo(() -> Commands.sequence(
          intake.setState(Intake.State.OFF),
          transfer.setState(Transfer.State.OFF)
        ).schedule());
      case PREPARE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN),
          amp.setState(Amp.State.IN),
          transfer.setState(Transfer.State.AMP),
          Commands.waitUntil(() -> !transfer.hasNote()).withTimeout(5.0),
          transfer.setState(Transfer.State.OFF),
          amp.setState(Amp.State.OFF),
          amp.setState(Amp.State.UP),
          amp.setState(Amp.State.OFF)
        ).finallyDo(() -> Commands.sequence(
          amp.setState(Amp.State.OFF),
          transfer.setState(Transfer.State.OFF)
        ).schedule());
      case PLACE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.UP),
          amp.setState(Amp.State.OUT),
          Commands.waitUntil(() -> !amp.hasNote()).withTimeout(5.0)
        ).finallyDo(() -> Commands.sequence(
          transfer.setState(Transfer.State.OFF),
          amp.setState(Amp.State.OFF)
        ).schedule());
      case LEAVE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN)
        ).finallyDo(() -> Commands.sequence(
          amp.setState(Amp.State.OFF)
        ).schedule());
      case PREPARE_SPEAKER:
        return Commands.sequence(
          shooter.setState(Shooter.State.IN),
          transfer.setState(Transfer.State.SHOOTER),
          Commands.waitUntil(() -> !transfer.hasNote()).withTimeout(5.0)
        ).finallyDo(() -> Commands.sequence(
          shooter.setState(Shooter.State.OFF),
          transfer.setState(Transfer.State.OFF)
        ).schedule());
      case SHOOT_SPEAKER:
        return Commands.sequence(
          shooter.setState(Shooter.State.SHOOT)
        ).finallyDo(() -> Commands.sequence(
          shooter.setState(Shooter.State.OFF)
        ).schedule());
      default:
        return Commands.run(() -> {});
    }
  }

  @Override
  public void periodic() {
    
  }
}
