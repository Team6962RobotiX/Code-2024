// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** An example command that uses an example subsystem. */

public class XBoxSwerve extends CommandBase {
  private final SwerveDrive drive;
  private final Supplier<XboxController> xboxSupplier;
  private final PIDController rotatePID = new PIDController(
      SwerveDriveConfig.TELEOP_ROTATE_PID[0],
      SwerveDriveConfig.TELEOP_ROTATE_PID[1],
      SwerveDriveConfig.TELEOP_ROTATE_PID[2]);
  
  private int n = 0;
  
  public XBoxSwerve(SwerveDrive drive, Supplier<XboxController> xboxSupplier) {
    this.drive = drive;
    this.xboxSupplier = xboxSupplier;

    rotatePID.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    XboxController controller = xboxSupplier.get();

    double xVelocity = -getControllerAxis(0) * SwerveDriveConfig.MAX_VELOCITY / 2.0;
    double yVelocity = -getControllerAxis(1) * SwerveDriveConfig.MAX_VELOCITY / 2.0;
    double rotateSpeed = Math.hypot(-getControllerAxis(4), -getControllerAxis(5));
    double targetAngle = (((-Math.atan2(-getControllerAxis(5), -getControllerAxis(4)) / Math.PI * 180.0) + 180.0 + 90.0) % 360.0) - 180.0;

    rotatePID.setP(SwerveDriveConfig.TELEOP_ROTATE_PID[0] * rotateSpeed);
    double angularVelocity = rotatePID.calculate(drive.getHeading(), targetAngle) * SwerveDriveConfig.FULL_POWER_ANGULAR_VELOCITY;


    if (Math.abs(angularVelocity) > SwerveDriveConfig.MAX_ANGULAR_VELOCITY / 2.0) {
      angularVelocity = SwerveDriveConfig.MAX_ANGULAR_VELOCITY / 2.0 * Math.signum(angularVelocity);
    }
    // drive.getModules()[0].getDriveMotor().set(1.0);

    // if (controller.getAButton()) {
    //   drive.getModules()[0].getDriveMotor().set(1.0);
    //   System.out.println(drive.getModules()[0].getDriveVelocity());
    // } else {
    //   drive.getModules()[0].getDriveMotor().set(0.0);
    // }

    if (controller.getBButton() || !controller.isConnected()) {
      drive.stopModules();
      return;
    }

    if (controller.getXButton()) {
      drive.groundModules();
      return;
    }

    if (controller.getYButton()) {
      drive.zeroHeading();
      return;
    }


    drive.fieldOrientedDrive(yVelocity, xVelocity, angularVelocity);
    
    n += 1;
    if (n % 10 == 0) {
      System.out.println(targetAngle);
    }
  }

  public double getControllerAxis(int id) {
    XboxController controller = xboxSupplier.get();
    double xValue = controller.getRawAxis(id);
    double yValue = 0;
    if (id == 0 || id == 1) { // Left Stick
      xValue = controller.getRawAxis(0);
      yValue = controller.getRawAxis(1);
    }
    if (id == 4 || id == 5) { // Right Stick
      xValue = controller.getRawAxis(4);
      yValue = controller.getRawAxis(5);
    }
    if (Math.hypot(xValue, yValue) < SwerveDriveConfig.CONTROLLER_DEADZONE) {
      return 0.0;
    }
    return controller.getRawAxis(id);
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