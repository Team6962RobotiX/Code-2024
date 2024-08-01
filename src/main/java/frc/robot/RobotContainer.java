// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.commands.autonomous.Autonomous;
import frc.robot.commands.drive.WheelRadiusCalibration;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
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


  // The robot's subsystems and commands
  private final SwerveDrive swerveDrive;
  
  private final Shooter shooter;
  private final Transfer transfer;
  private final Amp amp;
  private final Hang hang;
  private final Intake intake;
  private final RobotStateController stateController;
  private final LEDs ledStrip;
  // private final CollisionDetector collisionDetector;

  private static PowerDistribution PDH = new PowerDistribution(CAN.PDH, ModuleType.kRev);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    // Logger.autoLog("PDH", PDH);

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
    StatusChecks.addCheck(new SubsystemBase() {}, "Loop Time", () -> Robot.getComputeTime() <= 0.02);
    StatusChecks.addCheck(new SubsystemBase() {}, "3V3 Enabled", () -> RobotController.getEnabled3V3());
    StatusChecks.addCheck(new SubsystemBase() {}, "5V Enabled", () -> RobotController.getEnabled5V());
    StatusChecks.addCheck(new SubsystemBase() {}, "6V Enabled", () -> RobotController.getEnabled6V());
    StatusChecks.addCheck(new SubsystemBase() {}, "Sys Time Valid", () -> RobotController.isSystemTimeValid());
    StatusChecks.addCheck(new SubsystemBase() {}, "Amp Enabled", () -> Constants.ENABLED_SYSTEMS.ENABLE_AMP);
    StatusChecks.addCheck(new SubsystemBase() {}, "Dashboard Enabled", () -> Constants.ENABLED_SYSTEMS.ENABLE_DASHBOARD);
    StatusChecks.addCheck(new SubsystemBase() {}, "Drive Enabled", () -> Constants.ENABLED_SYSTEMS.ENABLE_DRIVE);
    StatusChecks.addCheck(new SubsystemBase() {}, "Hang Enabled", () -> Constants.ENABLED_SYSTEMS.ENABLE_HANG);
    StatusChecks.addCheck(new SubsystemBase() {}, "Intake Enabled", () -> Constants.ENABLED_SYSTEMS.ENABLE_INTAKE);
    StatusChecks.addCheck(new SubsystemBase() {}, "Shooter Enabled", () -> Constants.ENABLED_SYSTEMS.ENABLE_SHOOTER);
    StatusChecks.addCheck(new SubsystemBase() {}, "Transfer Enabled", () -> Constants.ENABLED_SYSTEMS.ENABLE_TRANSFER);

    swerveDrive = new SwerveDrive();
    shooter = new Shooter(swerveDrive);
    transfer = new Transfer();
    amp = new Amp();
    intake = new Intake();
    stateController = new RobotStateController(amp, swerveDrive, shooter, transfer, intake);
    hang = new Hang();
    ledStrip = new LEDs(stateController);
    // collisionDetector = new CollisionDetector();
    
    // Configure the trigger bindings
    Controls.configureBindings(stateController, swerveDrive, transfer, transfer.getInWheels(), transfer.getOutWheels(), shooter, shooter.getWheels(), shooter.getPivot(), amp, amp.getPivot(), amp.getWheels(), hang);

    SwerveDrive.printChoreoConfig();
    AprilTags.printConfig(Constants.LIMELIGHT.APRILTAG_CAMERA_POSES);

    Pathfinding.ensureInitialized();

    System.out.println("BEGINNING POSE TEST");

    Pose2d robotPose = new Pose2d(10, 14, Rotation2d.fromDegrees(74));
    Rotation2d angle = Rotation2d.fromDegrees(-67);
    int i = 3;

    System.out.print("Old: ");
    System.out.println(poseTestOld(robotPose, angle, i));
    System.out.print("New: ");
    System.out.println(poseTestNew(robotPose, angle, i));
  }

  private Pose2d poseTestOld(Pose2d robotPose, Rotation2d angle, int i) {
    return new Pose2d(
        ((i & 2) == 0 ? 1 : -1) * SWERVE_DRIVE.WHEELBASE / 2.0,
        ((i & 1) == 0 ? 1 : -1) * SWERVE_DRIVE.TRACKWIDTH / 2.0,
        angle
    ).relativeTo(new Pose2d(
        new Translation2d(),
        robotPose.getRotation().times(-1.0)
    )).relativeTo(new Pose2d(
        -robotPose.getX(),
        -robotPose.getY(),
        new Rotation2d()
    ));
  }

  private Pose2d poseTestNew(Pose2d robotPose, Rotation2d angle, int i) {
    return new Pose2d(
        robotPose.getX(),
        robotPose.getY(),
        robotPose.getRotation()
    ).plus(new Transform2d(
        ((i & 2) == 0 ? 1 : -1) * SWERVE_DRIVE.WHEELBASE / 2.0,
        ((i & 1) == 0 ? 1 : -1) * SWERVE_DRIVE.TRACKWIDTH / 2.0,
        angle
    ));
  }

  public Command getAutonomousCommand() {
    return new Autonomous(stateController, swerveDrive, AutonChooser.getNotes());
  }

  public static double getVoltage() {
    return RobotController.getBatteryVoltage();
  }

  public static double getTotalCurrent() {
    return PDH.getTotalCurrent();
  }

  public static PowerDistribution getPDH() {
    return PDH;
  }

  public void disabledPeriodic() {

  }

  public void disabledInit() {
    
  }

  public void testInit() {
    (new WheelRadiusCalibration(swerveDrive)).schedule();
  }
}
