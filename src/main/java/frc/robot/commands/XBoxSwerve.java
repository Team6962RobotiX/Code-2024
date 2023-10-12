// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.INPUT_MATH;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_MATH;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.Vector2D;

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
  private Vector2D newVelocity;
  private Vector2D currentVelocity;
  private Vector2D oldVelocity;

  private Vector2D leftStick;
  private Vector2D rightStick;
  private double leftTrigger;
  private double rightTrigger;

  // Calculate the maximum speeds we should drive at during teleop
  private double maxDriveVelocity = SwerveModule.powerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_DRIVE_POWER);
  private double slowDriveVelocity = SwerveModule.powerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_SLOW_DRIVE_POWER);
  private double maxRotateVelocity = SwerveDrive.wheelVelocityToRotationalVelocity(SwerveModule.powerToDriveVelocity(SWERVE_DRIVE.TELEOPERATED_ROTATE_POWER));

  private double tipCompensationSpeed = 0.0;

  // Acceleration limits to prevent tipping and skidding
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
    leftTrigger = controller.getLeftTriggerAxis();
    rightTrigger = controller.getRightTriggerAxis();
    leftStick = new Vector2D(-controller.getLeftX(), -controller.getLeftY());
    rightStick = new Vector2D(-controller.getRightX(), -controller.getRightY());
    
    // Deadbands
    leftTrigger = INPUT_MATH.nonLinear(leftTrigger);
    rightTrigger = INPUT_MATH.nonLinear(rightTrigger);
    leftStick = INPUT_MATH.circular(leftStick, 0.0, Math.PI / 4);
    rightStick = INPUT_MATH.circular(rightStick, 0.0, Math.PI / 4);

    // These variables we will eventually plug into the swerve drive command
    angularVelocity = 0.0;
    newVelocity.x = 0.0;
    newVelocity.y = 0.0;
    oldVelocity.set(currentVelocity);
    
    triggerRelativeMovement();
    
    rightStickAbsoluteRotation();
    
    bumperRotationLock();
    
    if (swerveDrive.getGyro().isConnected()) doFieldOrientedRotation();
    else fieldOrientedRotation = false;
    
    leftStickFieldOrientedDrive();

    slowDPadDrive();

    // Limit velocity magnitude
    double speed = Math.min(maxDriveVelocity, Math.hypot(newVelocity.x, newVelocity.y));
    double velocityAngle = Math.atan2(newVelocity.y, newVelocity.x);
    newVelocity.x = speed * Math.cos(velocityAngle);
    newVelocity.y = speed * Math.sin(velocityAngle);

    // Limit rotation speed
    angularVelocity = Math.abs(angularVelocity) > maxRotateVelocity ? maxRotateVelocity * Math.signum(angularVelocity) : angularVelocity;
    

    Vector2D oldPos = new Vector2D(currentVelocity).reverse().multiply(0.02);
    Vector2D reallyOldPos = new Vector2D(oldVelocity).reverse().multiply(0.02).add(oldPos);

    double[] circleOfMotion = SWERVE_MATH.circleFromPoints(new Vector2D(), oldPos, reallyOldPos);
    double centripetalForce = circleOfMotion[2] * SWERVE_DRIVE.ROBOT_MASS;
    Vector2D antiCentripetalForceVector = new Vector2D(circleOfMotion[0], circleOfMotion[1]).setMagnitude(centripetalForce);
    double newVelocityMagnitude = newVelocity.getMagnitude();
    newVelocity.add(antiCentripetalForceVector).setMagnitude(newVelocityMagnitude);

    // Limit acceleration
    angularVelocity = angularAccelerationLimiter.calculate(angularVelocity);
    Vector2D acceleration = new Vector2D(newVelocity).subtract(currentVelocity).divide(0.02);
    if (acceleration.getMagnitude() > SWERVE_DRIVE.TELEOPERATED_ACCELERATION) {
      acceleration.setMagnitude(SWERVE_DRIVE.TELEOPERATED_ACCELERATION);
    }
    currentVelocity.add(new Vector2D(acceleration).multiply(0.02));
    
    // Drive swerve
    swerveDrive.fieldOrientedDrive(currentVelocity.x, currentVelocity.y, angularVelocity);

    // Zero heading when Y is pressed
    if (controller.getYButton()) {
      swerveDrive.zeroHeading();
      targetRobotAngle = swerveDrive.getHeading();
    }

    // Rumble if acceleration is higher than expected (basically if we hit something)
    AHRS gyro = swerveDrive.getGyro();
    double gyroAcceleration = Math.sqrt(Math.pow(gyro.getRawAccelX(), 2) + Math.pow(gyro.getRawAccelY(), 2) + Math.pow(gyro.getRawAccelZ(), 2));
    double accelerationBeyondLimit = Math.max(0, gyroAcceleration - SWERVE_DRIVE.TELEOPERATED_ACCELERATION);
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
    boolean isRotating = Math.abs(Units.degreesToRadians(swerveDrive.getGyro().getRawGyroZ())) > SwerveDrive.wheelVelocityToRotationalVelocity(0.1);
    if (!fieldOrientedRotation && !isRotating && leftTrigger + rightTrigger == 0) {
      fieldOrientedRotation = true;
      targetRobotAngle = Units.degreesToRadians(swerveDrive.getHeading());
    }

    targetRobotAngle = SWERVE_MATH.clampRadians(targetRobotAngle);

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
      newVelocity.y += forwardSpeed * swerveDrive.getRotation2d().getCos();
      newVelocity.x += forwardSpeed * swerveDrive.getRotation2d().getSin();
    }
  }

  public void rightStickAbsoluteRotation() {
    // Some shenanigans to calculate the right stick angle
    double rightStickAngle = ((-Math.atan2(rightStick.y, rightStick.x) + Math.PI + Math.PI / 2) % (Math.PI * 2)) - Math.PI;
    double RStickMagnitude = Math.hypot(rightStick.x, rightStick.y);
    
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
    newVelocity.x += leftStick.x * maxDriveVelocity;
    newVelocity.y += leftStick.y * maxDriveVelocity;
  }

  public void slowDPadDrive() {
    // Slow drive fine control
    if (controller.getPOV() != -1) {
      newVelocity.x += -Math.sin(Units.degreesToRadians(controller.getPOV())) * slowDriveVelocity;
      newVelocity.y += Math.cos(Units.degreesToRadians(controller.getPOV())) * slowDriveVelocity;
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

    newVelocity.x += tipCompensationSpeed * Math.cos(tipDirection);
    newVelocity.y += tipCompensationSpeed * Math.sin(tipDirection);
  }
}