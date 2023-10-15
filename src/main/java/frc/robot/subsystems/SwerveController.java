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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
  private final SwerveDrive swerveDrive;
  
  public final double MAX_DRIVE_VELOCITY = SwerveModule.motorPowerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_DRIVE_POWER);
  public final double SLOW_DRIVE_VELOCITY = SwerveModule.motorPowerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_SLOW_DRIVE_POWER);
  public final double MAX_DRIVE_ACCELERATION = SWERVE_DRIVE.TELEOPERATED_ACCELERATION;
  public final double MAX_ANGULAR_VELOCITY = SwerveDrive.wheelVelocityToRotationalVelocity(SwerveModule.motorPowerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_ROTATE_POWER));
  public final double MAX_ANGULAR_ACCELERATION = SWERVE_DRIVE.TELEOPERATED_ANGULAR_ACCELERATION;

  private final ProfiledPIDController rotateController = new ProfiledPIDController(
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
  
  private Translation2d velocity = new Translation2d();
  private double angularVelocity = 0.0;

  private SlewRateLimiter angularAccelerationLimiter = new SlewRateLimiter(SWERVE_DRIVE.TELEOPERATED_ANGULAR_ACCELERATION);

  private double lastTimestamp = Timer.getFPGATimestamp();

  public SwerveController(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    rotateController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    if (doAbsoluteRotation) {
      angularVelocity = rotateController.calculate(swerveDrive.getHeading(), targetRobotAngle);
    } else {
      double currentAngularVelocity = Math.abs(Units.degreesToRadians(swerveDrive.getGyro().getRawGyroZ()));
      if (currentAngularVelocity < SwerveDrive.wheelVelocityToRotationalVelocity(SWERVE_DRIVE.VELOCITY_DEADBAND)) {
        doAbsoluteRotation = true;
      }
      targetRobotAngle = swerveDrive.getHeading();
    }
    limitAcceleration();

    boolean moving = false;
    for (SwerveModuleState moduleState : swerveDrive.getTargetModuleStates()) if (Math.abs(moduleState.speedMetersPerSecond) > SWERVE_DRIVE.VELOCITY_DEADBAND) moving = true;
    if (!moving) {
      swerveDrive.parkModules();
      return;
    }
    swerveDrive.fieldOrientedDrive(velocity.getX(), velocity.getY(), angularVelocity);
  }

  private void limitAcceleration() {
    double timeDelta = Timer.getFPGATimestamp() - lastTimestamp;
    ChassisSpeeds currentSpeeds = swerveDrive.getTargetChassisSpeeds();
    Translation2d currentVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    Translation2d velocityDelta = velocity.minus(currentVelocity);
    if (velocityDelta.getNorm() > MAX_DRIVE_ACCELERATION * timeDelta) velocityDelta = velocityDelta.div(velocityDelta.getNorm()).times(MAX_DRIVE_ACCELERATION * timeDelta);
    velocity = currentVelocity.plus(velocityDelta);
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
}