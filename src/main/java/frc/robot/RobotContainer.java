// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DEVICES;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.commands.vision.MoveToNote;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.RobotStateController.State;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.util.software.Logging.Logger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // private final LEDs ledStrip = new LEDs();

  // The robot's subsystems and commands
  private final CommandXboxController operatorController = new CommandXboxController(DEVICES.OPERATOR_XBOX_CONTROLLER);
  private final CommandXboxController driveController = new CommandXboxController(DEVICES.DRIVE_XBOX_CONTROLLER);
  private final SwerveDrive swerveDrive;
  
  private final Shooter shooter;
  private final Intake intake;
  private final Transfer transfer;
  private final Amp amp;

  private final RobotStateController stateController;

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    Logger.log("constants", this, Constants.class);
    Logger.autoLog("PDH", new PowerDistribution(CAN.PDH, ModuleType.kRev));
    Logger.startLog();

    swerveDrive = new SwerveDrive();
    shooter = new Shooter(swerveDrive);
    intake = new Intake();
    transfer = new Transfer();
    amp = new Amp();
    stateController = new RobotStateController(amp, swerveDrive, intake, shooter, transfer);

    swerveDrive.setDefaultCommand(new XBoxSwerve(swerveDrive, driveController.getHID()));

    // Configure the trigger bindings
    configureBindings();

    SwerveDrive.printChoreoConfig();
  }
  
  private void configureBindings() {

    // operatorController.a().onTrue(shooter.getPivot().setTargetAngle(Rotation2d.fromDegrees(5.0)));
    // operatorController.b().onTrue(shooter.getPivot().setTargetAngle(Rotation2d.fromDegrees(30.0)));

    operatorController.leftTrigger().whileTrue(stateController.setState(State.INTAKE));
    operatorController.rightStick().onTrue(stateController.setState(State.PREPARE_SPEAKER));
    // operatorController.leftBumper().whileTrue(stateController.setState(State.INTAKE));
    // operatorController.rightBumper().whileTrue(stateController.setState(State.INTAKE_OUT));

    // operatorController.leftTrigger().whileTrue(stateController.setState(State.PICKUP));
    // operatorController.rightStick().onTrue(stateController.setState(State.LOAD_SHOOTER));

    // operatorController.x().onTrue(stateController.setState(State.SHOOT_SPEAKER));
    operatorController.leftStick().onTrue(stateController.setState(State.PREPARE_AMP));
    operatorController.rightTrigger().whileTrue(stateController.setState(State.PLACE_AMP));
    operatorController.b().onTrue(amp.setState(Amp.State.DOWN));
    operatorController.a().whileTrue(stateController.setState(State.INTAKE_OUT));

    driveController.b().whileTrue(new MoveToNote("limelight-notes", swerveDrive, driveController));

    SwerveDrive.getField().getObject("Notes").setPoses(
      new Pose2d(Field.NOTE_POSITIONS[0], new Rotation2d()),
      new Pose2d(Field.NOTE_POSITIONS[1], new Rotation2d()),
      new Pose2d(Field.NOTE_POSITIONS[2], new Rotation2d()),
      new Pose2d(Field.NOTE_POSITIONS[3], new Rotation2d()),
      new Pose2d(Field.NOTE_POSITIONS[4], new Rotation2d()),
      new Pose2d(Field.NOTE_POSITIONS[5], new Rotation2d()),
      new Pose2d(Field.NOTE_POSITIONS[6], new Rotation2d()),
      new Pose2d(Field.NOTE_POSITIONS[7], new Rotation2d())
    );
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void disabledPeriodic() {

  }

  public void disabledInit() {
    
  }

  public void testInit() {
    
  }
}
