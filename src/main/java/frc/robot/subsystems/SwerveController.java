// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INPUT_MATH;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_MATH;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

/** An example command that uses an example subsystem. */

public class SwerveController extends SubsystemBase {
  private SwerveDrive swerveDrive;
  
  public final double MAX_DRIVE_VELOCITY = SwerveModule.motorPowerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_BOOST_DRIVE_POWER);
  public final double NOMINAL_DRIVE_VELOCITY = SwerveModule.motorPowerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_DRIVE_POWER);
  public final double SLOW_DRIVE_VELOCITY = SwerveModule.motorPowerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_SLOW_DRIVE_POWER);
  public final double MAX_DRIVE_ACCELERATION = SWERVE_DRIVE.TELEOPERATED_ACCELERATION;
  public final double MAX_ANGULAR_VELOCITY = SwerveDrive.wheelVelocityToRotationalVelocity(SwerveModule.motorPowerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_ROTATE_POWER));
  public final double MAX_ANGULAR_ACCELERATION = SWERVE_DRIVE.TELEOPERATED_ANGULAR_ACCELERATION;

  private ProfiledPIDController rotateController = new ProfiledPIDController(
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kP,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kI,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kD,
    new Constraints(
      MAX_ANGULAR_VELOCITY,
      MAX_ANGULAR_ACCELERATION
    )
  );

  private double targetRobotAngle = 0.0;
  private boolean doAbsoluteRotation = true;
  private int absoluteRotationCounter = 0;
  
  private Translation2d velocity = new Translation2d();
  private Translation2d oldVelocity = new Translation2d();
  private double angularVelocity = 0.0;

  private SlewRateLimiter angularAccelerationLimiter = new SlewRateLimiter(MAX_ANGULAR_ACCELERATION);

  private double lastTimestamp = Timer.getFPGATimestamp();

  public SwerveController(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    rotateController.enableContinuousInput(-Math.PI, Math.PI);
    rotateController.setTolerance(Units.degreesToRadians(1.0));
  }

  @Override
  public void periodic() {
    if (DriverStation.isAutonomous()) {
      return;
    }

    double currentAngularVelocity = swerveDrive.getMeasuredChassisSpeeds().omegaRadiansPerSecond;    
    if (doAbsoluteRotation) {
      angularVelocity = rotateController.calculate(swerveDrive.getHeading(), targetRobotAngle);
    } else {
      if (Math.abs(currentAngularVelocity) < SwerveDrive.wheelVelocityToRotationalVelocity(SWERVE_DRIVE.VELOCITY_DEADBAND) && ++absoluteRotationCounter > 10) {
        doAbsoluteRotation = true;
        absoluteRotationCounter = 0;
      }
      targetRobotAngle = swerveDrive.getHeading();
      rotateController.calculate(targetRobotAngle, new TrapezoidProfile.State(targetRobotAngle, currentAngularVelocity));
    }
    
    if (SwerveDrive.rotationalVelocityToWheelVelocity(Math.abs(angularVelocity)) < SWERVE_DRIVE.VELOCITY_DEADBAND) angularVelocity = 0.0;
    if (velocity.getNorm() < SWERVE_DRIVE.VELOCITY_DEADBAND) velocity = new Translation2d();

    limitAcceleration();
    
    boolean moving = false;
    moving = moving || SwerveDrive.rotationalVelocityToWheelVelocity(Math.abs(angularVelocity)) > SWERVE_DRIVE.VELOCITY_DEADBAND;
    moving = moving || velocity.getNorm() > SWERVE_DRIVE.VELOCITY_DEADBAND;

    for (SwerveModuleState moduleState : swerveDrive.getTargetModuleStates()) if (Math.abs(moduleState.speedMetersPerSecond) > SWERVE_DRIVE.VELOCITY_DEADBAND) moving = true;
    for (SwerveModuleState moduleState : swerveDrive.getMeasuredModuleStates()) if (Math.abs(moduleState.speedMetersPerSecond) > SWERVE_DRIVE.VELOCITY_DEADBAND) moving = true;
    if (!moving) {
      swerveDrive.parkModules();
      return;
    }
    swerveDrive.fieldOrientedDrive(velocity.getX(), velocity.getY(), angularVelocity);
    
    angularVelocity = 0.0;
    oldVelocity = new Translation2d(velocity.getX(), velocity.getY());
    velocity = new Translation2d();
  }

  private void limitAcceleration() {
    double timeDelta = Timer.getFPGATimestamp() - lastTimestamp;
    Translation2d acceleration = velocity.minus(oldVelocity).div(timeDelta);
    if (acceleration.getNorm() > MAX_DRIVE_ACCELERATION) acceleration = acceleration.div(acceleration.getNorm()).times(MAX_DRIVE_ACCELERATION);
    velocity = oldVelocity.plus(acceleration.times(timeDelta));
    angularVelocity = angularAccelerationLimiter.calculate(angularVelocity);
    lastTimestamp += timeDelta;
  }

  public void addVelocity(Translation2d addedVelocity) {
    velocity = velocity.plus(addedVelocity);
  }

  public void addAngularVelocity(double addedAngularVelocity) {
    angularVelocity += addedAngularVelocity;
    doAbsoluteRotation = false;
  }

  public void setTargetRobotAngle(double newTargetRobotAngle) {
    targetRobotAngle = newTargetRobotAngle;
    doAbsoluteRotation = true;
  }

  public void addVelocity(double addedVelocity, double direction) {
    velocity = velocity.plus(new Translation2d(
      addedVelocity * Math.cos(direction),
      addedVelocity * Math.sin(direction)
    ));
  }

  public Translation2d joystickToFieldMovement(Translation2d joystickMovement) {
    return new Translation2d(
      -joystickMovement.getY(),
      -joystickMovement.getX()
    );
  }

  public void stopModules() {
    swerveDrive.stopModules();
  }

  public void snapToNearest90() {
    setTargetRobotAngle(Math.round(targetRobotAngle / (Math.PI / 2)) * (Math.PI / 2));
  }

  public void zeroHeading() {
    swerveDrive.zeroHeading();
    targetRobotAngle = swerveDrive.getHeading();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
}