// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Field;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.vision.ApriltagPose;
import frc.robot.util.MathUtils;
import frc.robot.util.MathUtils.InputMath;

public class XBoxSwerve extends Command {
  private XboxController controller;
  private SwerveDrive swerveDrive;
  
  public final double MAX_DRIVE_VELOCITY = SwerveModule.calcWheelVelocity(SWERVE_DRIVE.TELEOPERATED_BOOST_POWER);
  public final double NOMINAL_DRIVE_VELOCITY = SwerveModule.calcWheelVelocity(SWERVE_DRIVE.TELEOPERATED_DRIVE_POWER);
  public final double FINE_TUNE_DRIVE_VELOCITY = SwerveModule.calcWheelVelocity(SWERVE_DRIVE.TELEOPERATED_FINE_TUNE_DRIVE_POWER);
  public final double NOMINAL_ANGULAR_VELOCITY = SwerveDrive.toAngular(SwerveModule.calcWheelVelocity(SWERVE_DRIVE.TELEOPERATED_ROTATE_POWER));
  public final double MAX_ANGULAR_VELOCITY = SwerveDrive.toAngular(MAX_DRIVE_VELOCITY); // TODO: use physics from constants file
  
  private Translation2d velocity = new Translation2d();
  private double angularVelocity = 0.0;
  
  public XBoxSwerve(SwerveDrive swerveDrive, Supplier<XboxController> xboxSupplier) {
    this.swerveDrive = swerveDrive;
    controller = xboxSupplier.get();
    // controller.setRumble(RumbleType.kBothRumble, 1.0);
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Disable drive if the controller disconnects
    if (!controller.isConnected()) {
      swerveDrive.stopModules();
      return;
    }

    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    Translation2d leftStick = new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    Translation2d rightStick = new Translation2d(controller.getRightX(), -controller.getRightY());
    
    if (RobotBase.isSimulation()) {
      leftStick = new Translation2d(controller.getRawAxis(0), -controller.getRawAxis(1));
      rightStick = new Translation2d(controller.getRawAxis(3), -controller.getRawAxis(4));
      leftTrigger = (controller.getRawAxis(5) + 1.0) / 2.0;
      rightTrigger = (controller.getRawAxis(4) + 1.0) / 2.0;
    }

    // Deadbands
    leftStick = InputMath.circular(leftStick, 0.05);

    angularVelocity += -rightStick.getX() * MathUtils.map(Math.max(leftTrigger, rightTrigger), 0, 1, NOMINAL_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    
    velocity = velocity.plus(leftStick.times(MathUtils.map(Math.max(leftTrigger, rightTrigger), 0, 1, NOMINAL_DRIVE_VELOCITY, MAX_DRIVE_VELOCITY)));

    if (controller.getPOV() != -1) {
      Translation2d povVelocity = new Translation2d(Math.cos(Units.degreesToRadians(controller.getPOV())) * FINE_TUNE_DRIVE_VELOCITY, -Math.sin(Units.degreesToRadians(controller.getPOV())) * FINE_TUNE_DRIVE_VELOCITY);
      velocity = velocity.plus(povVelocity);
    }

    // Zero heading when Y is pressed
    if (controller.getYButton()) {
      Rotation2d newHeading = new Rotation2d();
      Pose2d visionPose = ApriltagPose.getRobotPose2d();
      if (visionPose != null) {
        newHeading = visionPose.getRotation();
      } else if (!Constants.IS_BLUE_TEAM) {
        newHeading = newHeading.plus(Rotation2d.fromDegrees(180.0));
      }
      swerveDrive.resetGyroHeading(newHeading);
    }

    if (SwerveDrive.toLinear(Math.abs(angularVelocity)) < SWERVE_DRIVE.VELOCITY_DEADBAND) {
      angularVelocity = 0.0;
    }

    if (velocity.getNorm() < SWERVE_DRIVE.VELOCITY_DEADBAND) {
      velocity = new Translation2d();
    }

    if (controller.getAButton()) {
      swerveDrive.goToNearestPose(List.of(Field.AUTO_MOVE_POSITIONS.values().toArray(new Pose2d[] {})), controller).schedule();
    }

    swerveDrive.driveFieldRelative(velocity.getX(), velocity.getY(), angularVelocity);

    if (leftStick.getNorm() > 0.05 && (controller.getLeftBumper() || controller.getRightBumper())) {
      swerveDrive.setTargetHeading(leftStick.getAngle());
    }

    angularVelocity = 0.0;
    velocity = new Translation2d();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}