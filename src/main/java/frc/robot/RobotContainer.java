// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  private final XboxController controller = new XboxController(Devices.USB_XBOX_CONTROLLER);
  private final SwerveDrive drive = new SwerveDrive();
  // private final Limelight limelight = new Limelight(LimelightConfig.NAME);
  private final MotionRecorder recorder = new MotionRecorder(drive);
  private final Logger logger = new Logger(drive);
  // private final Testing tesing = new Testing();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(new XBoxSwerve(drive, () -> controller));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    new Trigger(controller::getAButton).onTrue(recorder.startRecording());
    new Trigger(controller::getBButton).onTrue(recorder.stopRecording());
  }

  public Command getAutonomousCommand() {

    List<Translation2d> positionData = new ArrayList<>();
    positionData.add(new Translation2d(0.0, 1.0));
    
    TrajectoryConfig TrajectoryConfig = new TrajectoryConfig(SwerveDriveConstants.AUTO_MAX_VELOCITY, SwerveDriveConstants.AUTO_MAX_ACCELERATION)
        .setKinematics(SwerveMath.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)),
      positionData,
      new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)),
      TrajectoryConfig);

    PIDController xController = new PIDController(
        SwerveDriveConstants.AUTO_X_PID[0],
        SwerveDriveConstants.AUTO_X_PID[1],
        SwerveDriveConstants.AUTO_X_PID[2]);
    PIDController yController = new PIDController(
        SwerveDriveConstants.AUTO_Y_PID[0],
        SwerveDriveConstants.AUTO_Y_PID[1],
        SwerveDriveConstants.AUTO_Y_PID[2]);
    ProfiledPIDController angleController = new ProfiledPIDController(
        SwerveDriveConstants.AUTO_ROTATE_PID[0],
        SwerveDriveConstants.AUTO_ROTATE_PID[1],
        SwerveDriveConstants.AUTO_ROTATE_PID[2],
        SwerveDriveConstants.AUTO_ANGLE_CONSTRAINTS);
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        drive::getPose,
        SwerveMath.getKinematics(),
        xController,
        yController,
        angleController,
        drive::setModuleStates,
        drive);
    // return null;

    return new SequentialCommandGroup(
        new InstantCommand(() -> drive.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand, new InstantCommand(() -> drive.stopModules()));
  }

  public void disabledPeriodic() {
  }

  public Logger getLogger() {
    return logger;
  }
}
