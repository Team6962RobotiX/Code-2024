package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Preferences;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.util.software.Logging.StatusChecks;

// This class is a subsystem that controls the state of the robot. It is used to coordinate the actions of the intake, shooter, transfer, and amp subsystems.

public class RobotStateController extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private Amp amp;
  private Shooter shooter;
  private Transfer transfer;
  private Intake intake;
  private DigitalInput beamBreakSensor;
  private Debouncer beamBreakDebouncer = new Debouncer(0.05);
  private boolean isAiming;
  private Debouncer shotDebouncer = new Debouncer(0.5);
  private State currentState;
  // private static ShuffleboardTab tab = Shuffleboard.getTab("Auto");
  // private static SimpleWidget hasNote = tab.add("has Note", true).withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(0, 0);



  public enum State {
    INTAKE,
    INTAKE_OUT,
    PREPARE_AMP,
    PLACE_AMP,
    LEAVE_AMP,
    AIM_SPEAKER,
    SHOOT_SPEAKER,
    SPIN_UP,
    PREPARE_SOURCE,
    INTAKE_SOURCE,
    PREPARE_TRAP,
    SHOOT_TRAP,
    CENTER_NOTE,
  }

  public RobotStateController(Amp amp, SwerveDrive swerveDrive, Shooter shooter, Transfer transfer, Intake intake) {
    this.swerveDrive = swerveDrive;
    this.amp = amp;
    this.shooter = shooter;
    this.transfer = transfer;
    this.intake = intake;
    beamBreakSensor = new DigitalInput(Constants.DIO.BEAM_BREAK);

    StatusChecks.addCheck(new SubsystemBase() {}, "Beam Break Sensor", () -> beamBreakSensor.get());
  }

  /**
   * Sets the state of the robot
   * @param state
   * @return
   */

  public Command setState(State state) {
    currentState = state;
    switch(state) {
      case INTAKE:
        return Commands.parallel(
          intake.setState(Intake.State.IN),
          transfer.setState(Transfer.State.SLOW_IN),
          amp.setState(Amp.State.DOWN)
        ).until(() -> hasNote()).raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND)).andThen(Commands.runOnce(() -> Controls.rumbleBoth().schedule()));
      case CENTER_NOTE:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN),
          amp.setState(Amp.State.IN).alongWith(
            transfer.setState(Transfer.State.AMP)
          ).until(() -> !hasNote()),
          amp.setState(Amp.State.OUT).alongWith(
            transfer.setState(Transfer.State.FROM_AMP),
            intake.setState(Intake.State.SLOW_OUT)
          ).until(() -> hasNote()),
          transfer.setState(Transfer.State.FROM_AMP).alongWith(intake.setState(Intake.State.SLOW_OUT)).until(() -> !hasNote()),
          transfer.setState(Transfer.State.SLOW_IN).alongWith(intake.setState(Intake.State.IN)).until(() -> hasNote())
        ).raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND)).andThen(Controls.rumbleBoth());
      case INTAKE_OUT:
        return transfer.setState(Transfer.State.OUT).alongWith(intake.setState(Intake.State.SLOW_OUT));
      case PREPARE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN),
          amp.setState(Amp.State.IN).alongWith(
            transfer.setState(Transfer.State.AMP)
          ).until(() -> !hasNote()),
          Commands.parallel(
            transfer.setState(Transfer.State.AMP),
            amp.setState(Amp.State.IN)
          ).withTimeout(0.25),
          amp.setState(Amp.State.UP)
        ).raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND)).andThen(Controls.rumbleBoth());
      case PLACE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.UP),
          amp.setState(Amp.State.OUT).withTimeout(2.0)
        ).raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND)).andThen(Controls.rumbleBoth());
      case LEAVE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN)
        );
      case AIM_SPEAKER:
        return shooter.setState(Shooter.State.AIM)
          .alongWith(Commands.runOnce(() -> isAiming = true))
          .alongWith(Controls.rumbleBoth(() -> canShoot()))
          .raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND))
          .alongWith(new ConditionalCommand(
            LEDs.setStateCommand(LEDs.State.BAD),
            Commands.runOnce(() -> {}),
            () -> !hasNote() && !RobotBase.isSimulation()
          )).finallyDo(() -> isAiming = false);
      case SHOOT_SPEAKER:
        return Commands.sequence(
          Commands.waitUntil(() -> canShoot()),
          transfer.setState(Transfer.State.SHOOTER_FAST).alongWith(intake.setState(Intake.State.IN)).until(() -> beamBreakSensor.get()),
          transfer.setState(Transfer.State.SHOOTER_SLOW)
        ).raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND));
      case SPIN_UP:
        return shooter.setState(Shooter.State.SPIN_UP);
      default:
        return Commands.run(() -> {});
    }
  }

  public State getState() {
    return currentState;
  }

  public boolean hasNote() {
    // if (RobotBase.isSimulation()) {
    //   return hasNote.getEntry().getBoolean(false);
    // }
    return beamBreakDebouncer.calculate(!beamBreakSensor.get());
  }

  public double getShooterVelocity() {
    return shooter.getVelocity();
  }

  public boolean isAiming() {
    return isAiming;
  }

  public boolean underStage() {
    // return true;
    return swerveDrive.underStage();
  }

  public Translation2d getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public double getShotChance() {
    // if (swerveDrive.underStage()) {
    //   return 0.0;
    // }
    // if (!hasNote() && !RobotBase.isSimulation()) {
    //   return 0.0;
    // }
    return shooter.getShotChance();
  }

  public boolean canShoot() {
    return shotDebouncer.calculate(getShotChance() == 1.0);
  }

  public boolean inRange() {
    return shooter.inRange();
  }

  @Override
  public void periodic() {
    shotDebouncer.calculate(getShotChance() == 1.0);
    beamBreakDebouncer.calculate(!beamBreakSensor.get());

    if (RobotState.isDisabled()) {
      LEDs.setState(LEDs.State.DISABLED);
    } else {
      LEDs.setState(LEDs.State.ENABLED);
    }
    
    if (swerveDrive.underStage()) {
      shooter.getShooterPivot().setMaxAngle(Preferences.SHOOTER_PIVOT.MAX_ANGLE_UNDER_STAGE);
      amp.getPivot().setMaxAngle(Preferences.AMP_PIVOT.MAX_ANGLE_UNDER_STAGE);
    } else {
      shooter.getShooterPivot().setMaxAngle(Preferences.SHOOTER_PIVOT.MAX_ANGLE);
      amp.getPivot().setMaxAngle(Preferences.AMP_PIVOT.MAX_ANGLE);
    }
  }
}
