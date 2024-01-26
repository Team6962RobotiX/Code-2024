// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DEVICES;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.Constants.NEO;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.DRIVE_MOTOR_PROFILE;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.PhotonLib;
import frc.robot.util.Logging.Logger;
import frc.robot.subsystems.vision.Camera;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands
  private final XboxController XboxController = new XboxController(DEVICES.USB_XBOX_CONTROLLER);
  private final SwerveDrive swerveDrive = new SwerveDrive();

  //Simulation only - getPose() does not work in real life
  private final Camera camera = new Camera("default", swerveDrive::getPose);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    Logger.log("constants", this, Constants.class);
    Logger.autoLog("PDH", new PowerDistribution(CAN.PDH, ModuleType.kRev));
    Logger.startLog();

    swerveDrive.setDefaultCommand(new XBoxSwerve(swerveDrive, () -> XboxController));
    //Positive y moves the camera left, Positive x moves the camera forward - TEMPORARY
    swerveDrive.resetPose(new Pose2d(new Translation2d(2, 2), new Rotation2d()));
    // Configure the trigger bindings
    configureBindings();
    
    SwerveDrive.printChoreoConfig();

  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return swerveDrive.followChoreoTrajectory("simple", true);
  }

  public void disabledPeriodic() {
  }
}
