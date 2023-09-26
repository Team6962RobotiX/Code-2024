// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** An example command that uses an example subsystem. */

public class XBoxSwerve extends CommandBase {
  private final SwerveDrive swerveDrive;
  private final Supplier<XboxController> xboxSupplier;
  private final PIDController rotatePID = new PIDController(
      SwerveDriveConstants.TELEOP_ROTATE_PID[0],
      SwerveDriveConstants.TELEOP_ROTATE_PID[1],
      SwerveDriveConstants.TELEOP_ROTATE_PID[2]);

  private double yVelocity = 0.0;
  private double xVelocity = 0.0;
  private double rotateVelocity = 0.0;

  private double targetRotateAngle = 0.0;

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

    double leftX = -controller.getLeftX();
    double leftY = -controller.getLeftY();
    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getLeftTriggerAxis();
    double rightX = -controller.getRightX();
    double rightY = -controller.getRightY();

    // For when doing simulation mode
    // rightX = -controller.getRawAxis(2);
    // rightY = -controller.getRawAxis(3);
    // leftTrigger = (controller.getRawAxis(5) + 1.0) / 2.0;
    // rightTrigger = (controller.getRawAxis(6) + 1.0) / 2.0;
    
    leftTrigger = InputMath.addLinearDeadzone(leftTrigger, SwerveDriveConstants.JOYSTICK_DEADZONE);
    rightTrigger = InputMath.addLinearDeadzone(rightTrigger, SwerveDriveConstants.JOYSTICK_DEADZONE);
    leftX = InputMath.addLinearDeadzone(leftX, SwerveDriveConstants.JOYSTICK_DEADZONE);
    leftY = InputMath.addLinearDeadzone(leftY, SwerveDriveConstants.JOYSTICK_DEADZONE);
    rightX = InputMath.addLinearDeadzone(rightX, SwerveDriveConstants.JOYSTICK_DEADZONE);
    rightY = InputMath.addLinearDeadzone(rightY, SwerveDriveConstants.JOYSTICK_DEADZONE);

    double LStickMagnitude = Math.hypot(leftX, leftY);
    double RStickMagnitude = Math.hypot(rightX, rightY);
    
    double maxDriveVelocity = SwerveMath.motorPowerToWheelVelocity(Constants.map(leftTrigger, 0.0, 1.0, SwerveDriveConstants.TELEOP_DRIVE_POWER, SwerveDriveConstants.TELEOP_DRIVE_BOOST_POWER));
    double maxRotateVelocity = SwerveMath.wheelVelocityToRotationalVelocity(SwerveMath.motorPowerToWheelVelocity(SwerveDriveConstants.TELEOP_ROTATE_POWER));
    
    double rightStickAngle = (Math.atan2(-rightY, rightX) + Math.PI / 2) % Math.PI;
    
    double rotateVelocity = 0.0;

    // If the triggers are used to do relative rotation
    if (leftTrigger + rightTrigger > 0) {
      double triggerDifference = -leftTrigger + rightTrigger;
      rotateVelocity = triggerDifference * maxRotateVelocity;
      
      // compensate for acceleration
      targetRotateAngle = Units.degreesToRadians(swerveDrive.getHeading()) + (rotateVelocity / (2 * SwerveDriveConstants.ROTATE_MAX_ACCELERATION));
    } else {

      // If the right stick is being used to do absolute rotation
      if (RStickMagnitude > 1.0 - SwerveDriveConstants.JOYSTICK_DEADZONE) {
        targetRotateAngle = rightStickAngle; 
      }

      rotateVelocity = rotatePID.calculate(
        Units.degreesToRadians(swerveDrive.getHeading()),
        targetRotateAngle);
      
      if (Math.abs(rotateVelocity) > maxRotateVelocity) {
        rotateVelocity = maxRotateVelocity * Math.signum(rotateVelocity);
      }
    }

    xVelocity = leftX * maxDriveVelocity;
    yVelocity = leftY * maxDriveVelocity;

    if (!controller.isConnected()) {
      swerveDrive.stopModules();
      return;
    }

    swerveDrive.fieldOrientedDrive(yVelocity, xVelocity, rotateVelocity);

    if (controller.getYButton()) {
      swerveDrive.zeroHeading();
      targetRotateAngle = swerveDrive.getHeading();
    }

    AHRS gyro = swerveDrive.getGyro();
    double acceleration = Math.sqrt(Math.pow(gyro.getRawAccelX(), 2) + Math.pow(gyro.getRawAccelY(), 2) + Math.pow(gyro.getRawAccelZ(), 2));
    double accelerationBeyondLimit = Math.max(0, acceleration - SwerveDriveConstants.DRIVE_MAX_ACCELERATION);
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