package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.util.software.Logging.StatusChecks;

// This class is a subsystem that controls the state of the robot. It is used to coordinate the actions of the intake, shooter, transfer, and amp subsystems.

public class RobotStateController extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private Amp amp;
  private Shooter shooter;
  private Transfer transfer;
  private DigitalInput beamBreakSensor;
  private Debouncer beamBreakDebouncer = new Debouncer(0.05);

  public enum State {
    INTAKE,
    INTAKE_OUT,
    PREPARE_AMP,
    PLACE_AMP,
    LEAVE_AMP,
    PREPARE_SPEAKER,
    AIM_SPEAKER,
    SHOOT_SPEAKER,
    SPIN_UP,
    PREPARE_SOURCE,
    INTAKE_SOURCE,
    PREPARE_TRAP,
    SHOOT_TRAP,
    CENTER_NOTE
  }

  public RobotStateController(Amp amp, SwerveDrive swerveDrive, Shooter shooter, Transfer transfer) {
    this.swerveDrive = swerveDrive;
    this.amp = amp;
    this.shooter = shooter;
    this.transfer = transfer;
    beamBreakSensor = new DigitalInput(Constants.DIO.BEAM_BREAK);
    StatusChecks.addCheck(new SubsystemBase() {}, "Beam Break Sensor", () -> beamBreakSensor.get());
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
          transfer.setState(Transfer.State.IN),
          amp.setState(Amp.State.DOWN)
        ).until(() -> hasNote()).andThen(Controls.rumble());
      case CENTER_NOTE:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN),
          amp.setState(Amp.State.IN).alongWith(
            transfer.setState(Transfer.State.AMP)
          ).until(() -> !hasNote()),
          amp.setState(Amp.State.OUT).alongWith(
            transfer.setState(Transfer.State.FROM_AMP)
          ).until(() -> hasNote()),
          transfer.setState(Transfer.State.FROM_AMP).until(() -> !hasNote()),
          transfer.setState(Transfer.State.IN).until(() -> hasNote())
        );
      case INTAKE_OUT:
        return transfer.setState(Transfer.State.OUT);
      case PREPARE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN),
          amp.setState(Amp.State.IN).alongWith(
            transfer.setState(Transfer.State.AMP)
          ).until(() -> !hasNote()),
          transfer.setState(Transfer.State.AMP).withTimeout(0.05),
          amp.setState(Amp.State.UP).raceWith(
            amp.setState(Amp.State.IN)
          )
        );
      case PLACE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.UP),
          amp.setState(Amp.State.OUT).withTimeout(2.0)
        ).andThen(Controls.rumble());
      case LEAVE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN)
        );
      case PREPARE_SPEAKER:
        return Commands.parallel(
          transfer.setState(Transfer.State.SHOOTER)
        ).until(() -> hasNote()).andThen(Controls.rumble());
      case AIM_SPEAKER:
        return shooter.setState(Shooter.State.AIM);
      case SHOOT_SPEAKER:
        return Commands.parallel(
          transfer.setState(Transfer.State.SHOOTER),
          shooter.setState(Shooter.State.SHOOT)
        );
      case SPIN_UP:
        return shooter.setState(Shooter.State.SPIN_UP);
      default:
        return Commands.run(() -> {});
    }
  }

  public boolean hasNote() {
    return beamBreakDebouncer.calculate(!beamBreakSensor.get());
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      LEDs.setState(LEDs.State.DISABLED);
    }
    
    if (hasNote()) {
      LEDs.setState(LEDs.State.HAS_NOTE);
    } else {
      LEDs.setState(LEDs.State.NO_NOTE);
    }

    // if (swerveDrive.underStage()) {
    //   shooter.getShooterPivot().setMaxAngle(Preferences.SHOOTER_PIVOT.MAX_ANGLE_UNDER_STAGE);
    //   amp.getPivot().setMaxAngle(Preferences.AMP_PIVOT.MAX_ANGLE_UNDER_STAGE);
    // } else {
    //   shooter.getShooterPivot().setMaxAngle(Preferences.SHOOTER_PIVOT.MAX_ANGLE);
    //   amp.getPivot().setMaxAngle(Preferences.AMP_PIVOT.MAX_ANGLE);
    // }
  }
}
