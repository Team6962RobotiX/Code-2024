// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
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
  private final Dashboard dashboard;
  private final Supplier<XboxController> xboxSupplier;
  private final PIDController rotatePID = new PIDController(
      SwerveDriveConstants.TELEOP_ROTATE_PID[0],
      SwerveDriveConstants.TELEOP_ROTATE_PID[1],
      SwerveDriveConstants.TELEOP_ROTATE_PID[2]);

  private double yVelocity = 0.0;
  private double xVelocity = 0.0;
  SlewRateLimiter slewLimiter = new SlewRateLimiter(SwerveDriveConstants.TELEOP_MAX_ACCELERATION);
  private double rotateVelocity = 0.0;

  private double targetRotateAngle = 0.0;

  private boolean absoluteRotation = false;

  public XBoxSwerve(SwerveDrive swerveDrive, Dashboard dashboard, Supplier<XboxController> xboxSupplier) {
    this.swerveDrive = swerveDrive;
    this.dashboard = dashboard;
    this.xboxSupplier = xboxSupplier;

    rotatePID.enableContinuousInput(Units.degreesToRadians(-180.0), Units.degreesToRadians(180.0));
    rotatePID.setTolerance(Units.degreesToRadians(SwerveDriveConstants.TELEOP_ROTATE_PID_TOLERANCE));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive, dashboard);
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
    double rightX = -controller.getRightX();
    double rightY = -controller.getRightY();

    if (Math.hypot(leftX, leftY) > 1.0 - SwerveDriveConstants.JOYSTICK_DEADZONE * 2) {
      if (Math.abs(leftX) < SwerveDriveConstants.JOYSTICK_DEADZONE * 2) {
        leftX = 0.0;
      }

      if (Math.abs(leftY) < SwerveDriveConstants.JOYSTICK_DEADZONE * 2) {
        leftY = 0.0;
      }
    }

    if (Math.hypot(rightX, rightY) > 1.0 - SwerveDriveConstants.JOYSTICK_DEADZONE * 2) {
      if (Math.abs(rightX) < SwerveDriveConstants.JOYSTICK_DEADZONE * 2) {
        rightX = 0.0;
      }

      if (Math.abs(rightY) < SwerveDriveConstants.JOYSTICK_DEADZONE * 2) {
        rightY = 0.0;
      }
    }

    if (Math.hypot(leftX, leftY) < SwerveDriveConstants.JOYSTICK_DEADZONE) {
      leftX = 0.0;
      leftY = 0.0;
    }

    if (Math.hypot(rightX, rightY) < SwerveDriveConstants.JOYSTICK_DEADZONE) {
      rightX = 0.0;
      rightY = 0.0;
    }

    double rightStickAngle = (((-Math.atan2(rightY, rightX) / Math.PI * 180.0) + 180.0 + 90.0) % 360.0) - 180.0;
    // rightStickAngle /= (360.0 / 16.0);
    // rightStickAngle = Math.round(rightStickAngle);
    // rightStickAngle *= (360.0 / 16.0);
    System.out.println(rightStickAngle);
    targetRotateAngle = rightStickAngle;

    // For when doing simulation mode
    // double rightX = -controller.getRawAxis(2);
    // double rightY = -controller.getRawAxis(3);
    // double leftTrigger = (controller.getRawAxis(5) + 1.0) / 2.0;

    double maxDriveVelocity = SwerveMath.motorPowerToWheelVelocity(Constants.map(leftTrigger, 0.0, 1.0, SwerveDriveConstants.TELEOP_DRIVE_POWER, SwerveDriveConstants.TELEOP_DRIVE_BOOST_POWER));
    double maxRotateVelocity = SwerveMath.wheelVelocityToRotationalVelocity(SwerveMath.motorPowerToWheelVelocity(SwerveDriveConstants.TELEOP_ROTATE_POWER));

    targetRotateAngle = ((targetRotateAngle % 360.0) + 360.0) % 360.0;

    // Left Stick
    xVelocity = leftX * maxDriveVelocity;
    yVelocity = leftY * maxDriveVelocity;

    rotateVelocity = rotatePID.calculate(
        Units.degreesToRadians(swerveDrive.getHeading()),
        Units.degreesToRadians(targetRotateAngle));
    if (Math.abs(rotateVelocity) > maxRotateVelocity) {
      rotateVelocity = maxRotateVelocity * Math.signum(rotateVelocity);
    }

    // Right Stick
    double LStickMagnitude = Math.hypot(leftX, leftY);
    double RStickMagnitude = Math.hypot(rightX, rightY);

    rotateVelocity *= RStickMagnitude;

    if (controller.getRightBumper()) {
      rotateVelocity = rightX * maxRotateVelocity;
    }

    if (!controller.isConnected()) {
      swerveDrive.stopModules();
      return;
    }

    double velocityUnfiltered = Math.hypot(xVelocity, yVelocity);
    double velocityFiltered = slewLimiter.calculate(velocityUnfiltered);

    xVelocity = xVelocity * velocityFiltered / velocityUnfiltered;
    yVelocity = yVelocity * velocityFiltered / velocityUnfiltered;

    swerveDrive.fieldOrientedDrive(yVelocity, xVelocity, rotateVelocity);

    if (controller.getLeftBumper()) {
      swerveDrive.robotOrientedDrive(yVelocity, xVelocity, rotateVelocity);
    }

    if (Math.hypot(yVelocity, xVelocity) < SwerveDriveConstants.VELOCITY_DEADZONE && Math.abs(rotateVelocity) < SwerveMath.wheelVelocityToRotationalVelocity(SwerveDriveConstants.VELOCITY_DEADZONE)) {
      swerveDrive.groundModules();
    }

    if (controller.getYButton()) {
      swerveDrive.zeroHeading();
      targetRotateAngle = swerveDrive.getHeading();
    }

    double avgWheelPower = 0.0;
    for (SwerveModule module : swerveDrive.getModules()) {
      avgWheelPower += SwerveMath.wheelVelocityToMotorPower(Math.abs(module.getVelocity()));
    }
    avgWheelPower /= 4.0;

    double rumble = Constants.map(Math.max(avgWheelPower - SwerveDriveConstants.TELEOP_DRIVE_POWER, 0), 0.0, SwerveDriveConstants.MOTOR_POWER_HARD_CAP - SwerveDriveConstants.TELEOP_DRIVE_POWER, 0, 1);

    controller.setRumble(RumbleType.kBothRumble, rumble);

    // if (controller.getAButton()) {
    //   dashboard.initialize();
    // }
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