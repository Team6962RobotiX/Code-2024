// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DEVICES;
import frc.robot.Constants.NEO;
import frc.robot.Presets.SHOOTER;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpPivot;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.amp.AmpWheels;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.FeedWheels;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.util.software.Logging.Logger;
import frc.robot.subsystems.RobotStateController.State;


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
    operatorController.y().onTrue(shooter.getFeedWheels().setState(FeedWheels.State.IN));


    operatorController.a().onTrue(amp.setState(Amp.State.OUT));
    operatorController.b().onTrue(amp.setState(Amp.State.IN));
    operatorController.rightBumper().whileTrue(intake.setState(Intake.State.IN));
    operatorController.leftStick().whileTrue(Commands.parallel(transfer.setState(Transfer.State.AMP), amp.setState(Amp.State.IN)));
    operatorController.leftStick().onFalse(Commands.parallel(transfer.setState(Transfer.State.OFF), amp.setState(Amp.State.OFF)));
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
