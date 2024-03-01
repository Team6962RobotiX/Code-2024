// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DEVICES;
import frc.robot.Constants.Constants.SHOOTER_PIVOT;
import frc.robot.commands.autonomous.Autonomous;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.commands.vision.MoveToNote;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.RobotStateController.State;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterMath;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferInWheels;
import frc.robot.subsystems.vision.AprilTags;
import frc.robot.util.software.Dashboard.AutonChooser;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // private final LEDs ledStrip = new LEDs();

  // The robot's subsystems and commands
  private final SwerveDrive swerveDrive;
  
  private final Shooter shooter;
  private final Transfer transfer;
  private final Amp amp;
  private final Hang hang;
  private final RobotStateController stateController;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    Logger.autoLog("PDH", new PowerDistribution(CAN.PDH, ModuleType.kRev));

    Logger.startLog();
    AutonChooser.init();
    
    LiveWindow.disableAllTelemetry();
    
    DriverStation.silenceJoystickConnectionWarning(true);

    StatusChecks.addCheck(new SubsystemBase() {}, "FMS Attached", () -> DriverStation.isFMSAttached());
    StatusChecks.addCheck(new SubsystemBase() {}, "DS Attached", () -> DriverStation.isDSAttached());
    StatusChecks.addCheck(new SubsystemBase() {}, "Joystick 0", () -> DriverStation.isJoystickConnected(0));
    StatusChecks.addCheck(new SubsystemBase() {}, "Joystick 1", () -> DriverStation.isJoystickConnected(1));
    StatusChecks.addCheck(new SubsystemBase() {}, "Battery Voltage", () -> RobotController.getBatteryVoltage() > 12.0);
    // StatusChecks.addCheck(new SubsystemBase() {}, "RSL", () -> RobotController.getRSLState());
    // StatusChecks.addCheck(new SubsystemBase() {}, "Sys Active", () -> RobotController.isSysActive());
    StatusChecks.addCheck(new SubsystemBase() {}, "3V3 Enabled", () -> RobotController.getEnabled3V3());
    StatusChecks.addCheck(new SubsystemBase() {}, "5V Enabled", () -> RobotController.getEnabled5V());
    StatusChecks.addCheck(new SubsystemBase() {}, "6V Enabled", () -> RobotController.getEnabled6V());
    StatusChecks.addCheck(new SubsystemBase() {}, "Sys Time Valid", () -> RobotController.isSystemTimeValid());

    swerveDrive = new SwerveDrive();
    shooter = new Shooter(swerveDrive);
    transfer = new Transfer();
    amp = new Amp();
    stateController = new RobotStateController(amp, swerveDrive, shooter, transfer);
    hang = new Hang(swerveDrive);
    
    // Configure the trigger bindings
    Controls.configureBindings(stateController, swerveDrive, transfer, transfer.getInWheels(), transfer.getOutWheels(), shooter, shooter.getWheels(), shooter.getPivot(), shooter.getFeedWheels(), amp, amp.getPivot(), amp.getWheels());

    SwerveDrive.printChoreoConfig();
    AprilTags.printConfig(Constants.LIMELIGHT.APRILTAG_CAMERA_POSES);
  }

  public Command getAutonomousCommand() {
    return new Autonomous(stateController, swerveDrive, AutonChooser.getNotes());
  }

  public void disabledPeriodic() {

  }

  public void disabledInit() {
    
  }

  public void testInit() {
    
  }
}
