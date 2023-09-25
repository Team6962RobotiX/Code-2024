// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.utils.Dashboard;
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

  private boolean absoluteRotation = false;

  public XBoxSwerve(SwerveDrive swerveDrive, Supplier<XboxController> xboxSupplier) {
    this.swerveDrive = swerveDrive;
    this.xboxSupplier = xboxSupplier;

    rotatePID.enableContinuousInput(Units.degreesToRadians(-180.0), Units.degreesToRadians(180.0));
    rotatePID.setTolerance(Units.degreesToRadians(SwerveDriveConstants.TELEOP_ROTATE_PID_TOLERANCE));
    // Use addRequirements() here to declare subsystem dependencies.
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
    double rightX = -controller.getRightX();
    double rightY = -controller.getRightY();

    leftX = InputMath.addLinearDeadzone(leftX, SwerveDriveConstants.JOYSTICK_DEADZONE);
    leftY = InputMath.addLinearDeadzone(leftY, SwerveDriveConstants.JOYSTICK_DEADZONE);
    rightX = InputMath.addLinearDeadzone(rightX, SwerveDriveConstants.JOYSTICK_DEADZONE);
    rightY = InputMath.addLinearDeadzone(rightY, SwerveDriveConstants.JOYSTICK_DEADZONE);

    // leftX = InputMath.addCirculuarDeadzone(new double[] { leftX, leftY }, SwerveDriveConstants.JOYSTICK_DEADZONE)[0];
    // leftY = InputMath.addCirculuarDeadzone(new double[] { leftX, leftY }, SwerveDriveConstants.JOYSTICK_DEADZONE)[1];
    // rightX = InputMath.addCirculuarDeadzone(new double[] { rightX, rightY }, SwerveDriveConstants.JOYSTICK_DEADZONE)[0];
    // rightY = InputMath.addCirculuarDeadzone(new double[] { rightX, rightY }, SwerveDriveConstants.JOYSTICK_DEADZONE)[1];

    double rightStickAngle = (((-Math.atan2(rightY, rightX) / Math.PI * 180.0) + 180.0 + 90.0) % 360.0) - 180.0;
    // rightStickAngle /= (360.0 / 16.0);
    // rightStickAngle = Math.round(rightStickAngle);
    // rightStickAngle *= (360.0 / 16.0);
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

    swerveDrive.fieldOrientedDrive(yVelocity, xVelocity, rotateVelocity);

    if (controller.getLeftBumper()) {
      swerveDrive.robotOrientedDrive(yVelocity, xVelocity, rotateVelocity);
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