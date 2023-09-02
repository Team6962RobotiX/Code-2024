// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
import frc.robot.subsystems.Drivetrain.SwerveModule;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** An example command that uses an example subsystem. */

public class XBoxSwerve extends CommandBase {
  private final SwerveDrive swerveDrive;
  private final Dashboard dashboard;
  private final Supplier<XboxController> xboxSupplier;
  private final PIDController rotatePID = new PIDController(
      SwerveDriveConfig.TELEOP_ROTATE_PID[0],
      SwerveDriveConfig.TELEOP_ROTATE_PID[1],
      SwerveDriveConfig.TELEOP_ROTATE_PID[2]);

  private double yVelocity = 0.0;
  private double xVelocity = 0.0;
  private double rotateVelocity = 0.0;

  private int n = 0;

  private double topSpeed = 0.0;

  public XBoxSwerve(SwerveDrive swerveDrive, Dashboard dashboard, Supplier<XboxController> xboxSupplier) {
    this.swerveDrive = swerveDrive;
    this.dashboard = dashboard;
    this.xboxSupplier = xboxSupplier;

    rotatePID.enableContinuousInput(Units.degreesToRadians(-180.0), Units.degreesToRadians(180.0));
    rotatePID.setTolerance(Units.degreesToRadians(SwerveDriveConfig.TELEOP_ROTATE_PID_TOLERANCE));
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

    // For when doing simulation mode
    // double rightX = -controller.getRawAxis(2);
    // double rightY = -controller.getRawAxis(3);
    // double leftTrigger = (controller.getRawAxis(5) + 1.0) / 2.0;

    double maxDriveVelocity = SwerveModule.motorPowerToWheelVelocity(Constants.map(leftTrigger, 0.0, 1.0, SwerveDriveConfig.TELEOP_DRIVE_POWER, SwerveDriveConfig.TELEOP_DRIVE_BOOST_POWER));
    double maxRotateVelocity = SwerveDrive.wheelVelocityToRotationalVelocity(SwerveModule.motorPowerToWheelVelocity(SwerveDriveConfig.TELEOP_ROTATE_POWER));

    // Left Stick
    double LStickMagnitude = Math.hypot(leftX, leftY);
    if (LStickMagnitude > SwerveDriveConfig.JOYSTICK_DEADZONE) {
      xVelocity = leftX * maxDriveVelocity;
      yVelocity = leftY * maxDriveVelocity;
    } else {
      xVelocity = 0.0;
      yVelocity = 0.0;
    }

    topSpeed = Math.max(topSpeed, swerveDrive.getGyroVelocity());
    
    // n += 1;
    // if (n % 10 == 0) {
    //   System.out.println(swerveDrive.getGyroVelocity());
    //   System.out.println(swerveDrive.getModules()[0].getDriveRPM());
    // }
    // System.out.println("current: " + Units.degreesToRadians(swerveDrive.getHeading()));
    // System.out.println("target: " + targetRotateAngle);

    // Right Stick
    double RStickMagnitude = Math.hypot(rightX, rightY);
    double targetRotateAngle = (((-Math.atan2(rightY, rightX) / Math.PI * 180.0) + 180.0 + 90.0) % 360.0) - 180.0;
    if (RStickMagnitude > SwerveDriveConfig.JOYSTICK_DEADZONE) {
      rotateVelocity = rotatePID.calculate(
          Units.degreesToRadians(swerveDrive.getHeading()),
          Units.degreesToRadians(targetRotateAngle));
      if (Math.abs(rotateVelocity) > maxRotateVelocity) {
        rotateVelocity = maxRotateVelocity * Math.signum(rotateVelocity);
      }
      rotateVelocity *= RStickMagnitude;
    } else {
      rotateVelocity = 0.0;
    }

    if (!controller.isConnected()) {
      swerveDrive.stopModules();
      return;
    }

    if (controller.getXButton()) {
      swerveDrive.groundModules();
      return;
    }

    if (controller.getYButton()) {
      swerveDrive.zeroHeading();
      // dashboard.initialize();
    }

    double wheelPower = SwerveModule.wheelVelocityToMotorPower(Math.hypot(Math.hypot(xVelocity, yVelocity), swerveDrive.rotationalVelocityToWheelVelocity(rotateVelocity)));

    if (wheelPower > SwerveDriveConfig.MOTOR_POWER_HARD_CAP) {
      controller.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      controller.setRumble(RumbleType.kBothRumble, 0.0);
    }

    // if (controller.getAButton()) {
    //   dashboard.initialize();
    // }

    if (controller.getLeftBumper()) {
      swerveDrive.robotOrientedDrive(yVelocity, xVelocity, rightX * maxRotateVelocity);
      return;
    }

    swerveDrive.fieldOrientedDrive(yVelocity, xVelocity, rotateVelocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotatePID.reset();
    rotatePID.setP(SwerveDriveConfig.TELEOP_ROTATE_PID[0]);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}