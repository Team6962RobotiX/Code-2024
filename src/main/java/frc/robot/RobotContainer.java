// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DEVICES;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.amp.AmpPivot;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.amp.AmpWheels;
import frc.robot.subsystems.amp.AmpWheels.AmpState;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.shooter.FeedWheels;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.TransferWheels;
import frc.robot.util.Logging.Logger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final LEDs ledStrip = new LEDs();

  // The robot's subsystems and commands
  private final CommandXboxController operatorController = new CommandXboxController(DEVICES.OPERATOR_XBOX_CONTROLLER);
  private final CommandXboxController driveController = new CommandXboxController(DEVICES.DRIVE_XBOX_CONTROLLER);
  // private final SwerveDrive swerveDrive = new SwerveDrive();
  // private final Shooter shooter = new Shooter(swerveDrive);
  
  private final SendableChooser<Command> calibrationChooser = new SendableChooser<>();
  private DutyCycleEncoder encoder;
  private final IntakeWheels intake = new IntakeWheels();
  private final TransferWheels transfer = new TransferWheels();
  private final AmpWheels amp = new AmpWheels();
  private final FeedWheels feedWheels = new FeedWheels();
  // private final AmpWheels ampPivot = new AmpPivot();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    Logger.log("constants", this, Constants.class);
    Logger.autoLog("PDH", new PowerDistribution(CAN.PDH, ModuleType.kRev));
    Logger.startLog();

    // swerveDrive.setDefaultCommand(new XBoxSwerve(swerveDrive, driveController.getHID()));
    
    // calibrationChooser.setDefaultOption("Calibrate Drive Motor (FL)", swerveDrive.modules[0].calibrateDriveMotor());
    // calibrationChooser.setDefaultOption("Calibrate Steer Motor (FL)", swerveDrive.modules[0].calibrateSteerMotor());
    // SmartDashboard.putData("Swerve Module Calibration", calibrationChooser);

    // Configure the trigger bindings
    configureBindings();

    SwerveDrive.printChoreoConfig();

    encoder = new DutyCycleEncoder(Constants.DIO.AMP_PIVOT);
  }

  private void configureBindings() {
    Command ampCommand = Commands.parallel(Commands.startEnd(() -> transfer.setState(TransferWheels.TransferState.AMP), 
                                                             () -> transfer.setState(TransferWheels.TransferState.OFF)),
                                           Commands.startEnd(() -> amp.setState(AmpWheels.AmpState.IN), 
                                                             () -> amp.setState(AmpWheels.AmpState.OFF)));
    Command shooterCommand = Commands.parallel(Commands.startEnd(() -> transfer.setState(TransferWheels.TransferState.SHOOTER),
                                                                 () -> transfer.setState(TransferWheels.TransferState.OFF)),
                                               Commands.startEnd(() -> feedWheels.setState(FeedWheels.ShooterState.FORWARD),
                                                                 () -> feedWheels.setState(FeedWheels.ShooterState.OFF)));
    operatorController.leftTrigger(0.1).whileTrue(Commands.startEnd(() -> amp.setState(AmpWheels.AmpState.OUT), () -> amp.setState(AmpWheels.AmpState.OFF)));
    operatorController.rightTrigger(0.1).whileTrue(Commands.startEnd(() -> intake.setState(IntakeWheels.IntakeState.IN), () -> intake.setState(IntakeWheels.IntakeState.OFF)));
    operatorController.leftStick().whileTrue(ampCommand);
    operatorController.rightStick().whileTrue(shooterCommand);
    // operatorController.rightBumper().whileTrue(Commands.startEnd(() -> transfer.setState(TransferWheels.TransferState.SHOOTER), () -> transfer.setState(TransferWheels.TransferState.OFF)));
  }
    public Command getAutonomousCommand() {
    // return swerveDrive.goTo(new Translation2d(5.0, 5.0), Rotation2d.fromDegrees(90.0));
    // return swerveDrive.followChoreoTrajectory("simple", true);
    return null;
  }

  public void disabledPeriodic() {
    
    //System.out.println(encoder.getAbsolutePosition());
  }

  public void testInit() {
    // calibrationChooser.getSelected().schedule();
  }
}
