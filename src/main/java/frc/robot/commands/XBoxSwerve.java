// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.InputMath;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SwerveMath;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.SwerveModule;

/** An example command that uses an example subsystem. */

public class XBoxSwerve extends CommandBase {
  private XboxController controller;
  
  private final SwerveDrive swerveDrive;
  private final PIDController rotatePID = new PIDController(
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kP,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kI,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kD
  );

  // What angle we want the robot to face
  private double targetRobotAngle = 0.0;
  private boolean fieldOrientedRotation = true;
  
  private double angularVelocity = 0.0;
  private double xVelocity = 0.0;
  private double yVelocity = 0.0;

  private double leftX;
  private double leftY;
  private double rightX;
  private double rightY;
  private double leftTrigger;
  private double rightTrigger;

  // Calculate the maximum speeds we should drive at during teleop
  private double maxDriveVelocity = SwerveModule.powerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_DRIVE_POWER);
  private double slowDriveVelocity = SwerveModule.powerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_SLOW_DRIVE_POWER);
  private double maxRotateVelocity = SwerveDrive.wheelVelocityToRotationalVelocity(SwerveModule.powerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_ROTATE_POWER));

  private double tipCompensationSpeed = 0.0;

  // Acceleration limits to prevent tipping and skidding
  private SlewRateLimiter xAccelerationLimiter = new SlewRateLimiter(SWERVE_DRIVE.TELEOPERATED_ACCELERATION);
  private SlewRateLimiter yAccelerationLimiter = new SlewRateLimiter(SWERVE_DRIVE.TELEOPERATED_ACCELERATION);
  private SlewRateLimiter angularAccelerationLimiter = new SlewRateLimiter(SWERVE_DRIVE.TELEOPERATED_ANGULAR_ACCELERATION);

  public XBoxSwerve(SwerveDrive swerveDrive, Supplier<XboxController> xboxSupplier) {
    this.swerveDrive = swerveDrive;
    controller = xboxSupplier.get();

    rotatePID.enableContinuousInput(-Math.PI, Math.PI);
    
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

    // Get all inputs (some need to be reversed)
    leftX = -controller.getLeftX();
    leftY = -controller.getLeftY();
    leftTrigger = controller.getLeftTriggerAxis();
    rightTrigger = controller.getRightTriggerAxis();
    rightX = -controller.getRightX();
    rightY = -controller.getRightY();

    // These variables we will eventually plug into the swerve drive command
    angularVelocity = 0.0;
    xVelocity = 0.0;
    yVelocity = 0.0;
    
    triggerRelativeMovement();
    
    rightStickAbsoluteRotation();
    
    bumperRotationLock();
    
    if (swerveDrive.getGyro().isConnected()) doFieldOrientedRotation();
    else fieldOrientedRotation = false;

    leftStickFieldOrientedDrive();

    slowDPadDrive();

    // Limit velocity magnitude
    double speed = Math.min(maxDriveVelocity, Math.hypot(xVelocity, yVelocity));
    double velocityAngle = Math.atan2(yVelocity, xVelocity);
    xVelocity = speed * Math.cos(velocityAngle);
    yVelocity = speed * Math.sin(velocityAngle);

    // Limit rotation speed
    angularVelocity = Math.abs(angularVelocity) > maxRotateVelocity ? maxRotateVelocity * Math.signum(angularVelocity) : angularVelocity;

    // Limit acceleration
    xVelocity = xAccelerationLimiter.calculate(xVelocity);
    yVelocity = yAccelerationLimiter.calculate(yVelocity);
    angularVelocity = angularAccelerationLimiter.calculate(angularVelocity);

    // Drive swerve
    swerveDrive.fieldOrientedDrive(yVelocity, xVelocity, angularVelocity);

    // Zero heading when Y is pressed
    if (controller.getYButton()) {
      swerveDrive.zeroHeading();
      targetRobotAngle = swerveDrive.getHeading();
    }

    // Rumble if acceleration is higher than expected (basically if we hit something)
    AHRS gyro = swerveDrive.getGyro();
    double acceleration = Math.sqrt(Math.pow(gyro.getRawAccelX(), 2) + Math.pow(gyro.getRawAccelY(), 2) + Math.pow(gyro.getRawAccelZ(), 2));
    double accelerationBeyondLimit = Math.max(0, acceleration - SWERVE_DRIVE.TELEOPERATED_ACCELERATION);
    double normalizedRumble = accelerationBeyondLimit / (accelerationBeyondLimit + 1);
    controller.setRumble(RumbleType.kBothRumble, normalizedRumble);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotatePID.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void doFieldOrientedRotation() {
    boolean isRotating = Units.degreesToRadians(swerveDrive.getGyro().getRawGyroZ()) > SwerveDrive.wheelVelocityToRotationalVelocity(SWERVE_DRIVE.VELOCITY_DEADBAND);
    if (!fieldOrientedRotation && !isRotating) {
      fieldOrientedRotation = true;
      targetRobotAngle = Units.degreesToRadians(swerveDrive.getHeading());
    }

    targetRobotAngle = SwerveMath.clampRadians(targetRobotAngle);

    if (fieldOrientedRotation) {
      // Calculate the angular velocity we need to rotate to the target angle
      angularVelocity = rotatePID.calculate(Units.degreesToRadians(swerveDrive.getHeading()), targetRobotAngle);
    }
  }

  public void triggerRelativeMovement() {
    if (leftTrigger + rightTrigger > 0) {
      fieldOrientedRotation = false;
      // Angular movement for each trigger
      double triggerDifference = leftTrigger + -rightTrigger;
      angularVelocity = triggerDifference * maxRotateVelocity;

      // Forward movement if both triggers are pressed
      double forwardSpeed = (Math.min(leftTrigger, rightTrigger)) * maxDriveVelocity;
      yVelocity += forwardSpeed * swerveDrive.getRotation2d().getCos();
      xVelocity += forwardSpeed * swerveDrive.getRotation2d().getSin();
    }
  }

  public void rightStickAbsoluteRotation() {
    // Some shenanigans to calculate the right stick angle
    double rightStickAngle = ((-Math.atan2(rightY, rightX) + Math.PI + Math.PI / 2) % (Math.PI * 2)) - Math.PI;
    double RStickMagnitude = Math.hypot(rightX, rightY);
    
    if (RStickMagnitude > 0.8) {
      fieldOrientedRotation = true;
      targetRobotAngle = rightStickAngle;
    }
  }

  public void bumperRotationLock() {
    // Rotate to nearest 90 degrees when any bumpers are pressed
    if (controller.getLeftBumper() || controller.getRightBumper()) {
      fieldOrientedRotation = true;
      targetRobotAngle = Math.round(targetRobotAngle / (Math.PI / 2)) * (Math.PI / 2);
    }
  }

  public void leftStickFieldOrientedDrive() {
    // Left stick field oriented drive
    xVelocity += leftX * maxDriveVelocity;
    yVelocity += leftY * maxDriveVelocity;
  }

  public void slowDPadDrive() {
    // Slow drive fine control
    if (controller.getPOV() != -1) {
      xVelocity += -Math.sin(Units.degreesToRadians(controller.getPOV())) * slowDriveVelocity;
      yVelocity += Math.cos(Units.degreesToRadians(controller.getPOV())) * slowDriveVelocity;
    }
  }

  public void tipCompensation() {
    AHRS gyro = swerveDrive.getGyro();

    double xTip = Units.degreesToRadians(gyro.getPitch()) * Math.cos(swerveDrive.getRotation2d().getRadians());
    double yTip = Units.degreesToRadians(gyro.getRoll()) * Math.sin(swerveDrive.getRotation2d().getRadians());

    double tipDirection = Math.atan2(yTip, xTip);
    double tipRadians = Math.hypot(xTip, yTip);

    if (tipRadians < SWERVE_DRIVE.TIP_COMPENSATION_MIN_TILT) {
      tipCompensationSpeed = 0.0;
      return;
    }

    double acceleration = tipRadians / Math.PI * SWERVE_DRIVE.AUTONOMOUS_ACCELERATION;
    
    tipCompensationSpeed += acceleration * 0.02;

    xVelocity += tipCompensationSpeed * Math.cos(tipDirection);
    yVelocity += tipCompensationSpeed * Math.sin(tipDirection);
  }
}