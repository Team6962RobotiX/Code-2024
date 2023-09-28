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
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveMath;
import frc.robot.subsystems.SwerveDrive;

/** An example command that uses an example subsystem. */

public class XBoxSwerve extends CommandBase {
  private final SwerveDrive swerveDrive;
  private final Supplier<XboxController> xboxSupplier;
  private final PIDController rotatePID = new PIDController(
      SwerveDriveConstants.TELEOP_ROTATE_PID[0],
      SwerveDriveConstants.TELEOP_ROTATE_PID[1],
      SwerveDriveConstants.TELEOP_ROTATE_PID[2]);

  // What angle we want the robot to face
  private double targetRobotAngle = 0.0;

  // Acceleration limits to prevent tipping and skidding
  private SlewRateLimiter xAccelerationLimiter = new SlewRateLimiter(SwerveDriveConstants.TELEOP_MAX_ACCELERATION);
  private SlewRateLimiter yAccelerationLimiter = new SlewRateLimiter(SwerveDriveConstants.TELEOP_MAX_ACCELERATION);
  private SlewRateLimiter angularAccelerationLimiter = new SlewRateLimiter(SwerveDriveConstants.TELEOP_MAX_ANGULAR_ACCELERATION);

  public XBoxSwerve(SwerveDrive swerveDrive, Supplier<XboxController> xboxSupplier) {
    this.swerveDrive = swerveDrive;
    this.xboxSupplier = xboxSupplier;

    rotatePID.enableContinuousInput(-Math.PI, Math.PI);
    rotatePID.setTolerance(Units.degreesToRadians(SwerveDriveConstants.TELEOP_ROTATE_PID_TOLERANCE));

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    XboxController controller = xboxSupplier.get();

    // Disable drive if the controller disconnects
    if (!controller.isConnected()) {
      swerveDrive.stopModules();
      return;
    }

    // Get all inputs (some need to be reversed)
    double leftX = -controller.getLeftX();
    double leftY = -controller.getLeftY();
    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    double rightX = -controller.getRightX();
    double rightY = -controller.getRightY();

    // For when doing simulation mode
    // rightX = -controller.getRawAxis(2);
    // rightY = -controller.getRawAxis(3);
    // leftTrigger = (controller.getRawAxis(5) + 1.0) / 2.0;
    // rightTrigger = (controller.getRawAxis(6) + 1.0) / 2.0;
    
    // Add deadbands to prevent misinputs and easier 90 degree alignments on joysticks
    leftTrigger = InputMath.addLinearDeadband(leftTrigger, SwerveDriveConstants.JOYSTICK_DEADBAND / 2);
    rightTrigger = InputMath.addLinearDeadband(rightTrigger, SwerveDriveConstants.JOYSTICK_DEADBAND / 2);
    leftX = InputMath.addLinearDeadband(leftX, SwerveDriveConstants.JOYSTICK_DEADBAND);
    leftY = InputMath.addLinearDeadband(leftY, SwerveDriveConstants.JOYSTICK_DEADBAND);
    rightX = InputMath.addLinearDeadband(rightX, SwerveDriveConstants.JOYSTICK_DEADBAND);
    rightY = InputMath.addLinearDeadband(rightY, SwerveDriveConstants.JOYSTICK_DEADBAND);

    // How far is each joystick from the center?
    double LStickMagnitude = Math.hypot(leftX, leftY);
    double RStickMagnitude = Math.hypot(rightX, rightY);
    
    // Calculate the maximum speeds we should drive at during teleop
    double maxDriveVelocity = SwerveMath.motorPowerToWheelVelocity(SwerveDriveConstants.TELEOP_DRIVE_POWER);
    double slowDriveVelocity = SwerveMath.motorPowerToWheelVelocity(SwerveDriveConstants.TELEOP_SLOW_DRIVE_POWER);
    double maxRotateVelocity = SwerveMath.wheelVelocityToRotationalVelocity(SwerveMath.motorPowerToWheelVelocity(SwerveDriveConstants.TELEOP_ROTATE_POWER));
    
    // Some shenanigans to calculate the right stick angle
    double rightStickAngle = ((-Math.atan2(rightY, rightX) + Math.PI + Math.PI / 2) % (Math.PI * 2)) - Math.PI;
    
    // These variables we will eventually plug into the swerve drive command
    double angularVelocity = 0.0;
    double xVelocity = 0.0;
    double yVelocity = 0.0;

    // If the triggers are used to do relative rotation
    if (leftTrigger + rightTrigger > 0) {
      // Angular movement for each trigger
      double triggerDifference = leftTrigger + -rightTrigger;
      angularVelocity = triggerDifference * maxRotateVelocity;
      // Map the velocity so the minimum value is the minimum speed we can move at
      angularVelocity = InputMath.mapBothSides(angularVelocity, 0, maxRotateVelocity, SwerveMath.wheelVelocityToRotationalVelocity(SwerveDriveConstants.VELOCITY_DEADBAND), maxRotateVelocity);
      
      // Forward movement if both triggers are pressed
      double forwardSpeed = (Math.min(leftTrigger, rightTrigger)) * maxDriveVelocity;
      yVelocity += forwardSpeed * swerveDrive.getRotation2d().getCos();
      xVelocity += forwardSpeed * swerveDrive.getRotation2d().getSin();

      // Calculate TargetRobotAngle, compensating for acceleration
      double currentAngularVelocity = swerveDrive.getTargetChassisSpeeds().omegaRadiansPerSecond;
      double timeToStop = currentAngularVelocity / (SwerveDriveConstants.TELEOP_MAX_ANGULAR_ACCELERATION * Math.signum(angularVelocity));
      double radiansToStop = (currentAngularVelocity * timeToStop) + (0.5 * SwerveDriveConstants.TELEOP_MAX_ANGULAR_ACCELERATION * Math.signum(angularVelocity) * Math.pow(timeToStop, 2));
      targetRobotAngle = Units.degreesToRadians(swerveDrive.getHeading()) + (radiansToStop);
    } else {

      // If the right stick is being used to do absolute rotation
      if (RStickMagnitude > 1.0 - SwerveDriveConstants.JOYSTICK_DEADBAND) {
        targetRobotAngle = rightStickAngle; 
      }

      // Rotate to nearest 90 degrees when any bumpers are pressed
      if (controller.getLeftBumper() || controller.getRightBumper()) {
        targetRobotAngle = Math.round(targetRobotAngle / (Math.PI / 2)) * (Math.PI / 2);
      }
      
      // Calculate the angular velocity we need to rotate to the target angle
      angularVelocity = rotatePID.calculate(
        Units.degreesToRadians(swerveDrive.getHeading()),
        targetRobotAngle);
    }

    // Left stick field oriented drive
    xVelocity += leftX * maxDriveVelocity;
    yVelocity += leftY * maxDriveVelocity;

    // Map the velocity so the minimum value is the minimum speed we can move at
    xVelocity = InputMath.mapBothSides(xVelocity, 0, maxDriveVelocity, SwerveDriveConstants.VELOCITY_DEADBAND, maxRotateVelocity);
    yVelocity = InputMath.mapBothSides(yVelocity, 0, maxDriveVelocity, SwerveDriveConstants.VELOCITY_DEADBAND, maxRotateVelocity);

    // Slow drive fine control
    if (controller.getPOV() != -1) {
      xVelocity += -Math.sin(Units.degreesToRadians(controller.getPOV())) * slowDriveVelocity;
      yVelocity += Math.cos(Units.degreesToRadians(controller.getPOV())) * slowDriveVelocity;
    }
    
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
    double accelerationBeyondLimit = Math.max(0, acceleration - SwerveDriveConstants.TELEOP_MAX_ACCELERATION);
    double normalizedRumble = accelerationBeyondLimit / (accelerationBeyondLimit + 1);
    controller.setRumble(RumbleType.kBothRumble, normalizedRumble);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotatePID.reset();
    rotatePID.setP(SwerveDriveConstants.TELEOP_ROTATE_PID[0]);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}