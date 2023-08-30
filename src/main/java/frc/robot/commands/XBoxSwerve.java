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
  
  public XBoxSwerve(SwerveDrive drive, Supplier<XboxController> xboxSupplier) {
    this.drive = drive;
    this.xboxSupplier = xboxSupplier;

    rotatePID.enableContinuousInput(0, 360);
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

    double xVelocity = getControllerAxis(0) * SwerveDriveConfig.MAX_VELOCITY;
    double yVelocity = getControllerAxis(1) * SwerveDriveConfig.MAX_VELOCITY;
    double rotateSpeed = Math.hypot(getControllerAxis(2), getControllerAxis(3));
    double targetAngle = ((Math.atan2(getControllerAxis(3), getControllerAxis(2)) / Math.PI * 180) + 360 + 90) % 360;

    rotatePID.setP(SwerveDriveConfig.TELEOP_ROTATE_PID[0] * rotateSpeed);
    double angularVelocity = rotatePID.calculate(drive.getHeading(), targetAngle) * SwerveDriveConfig.FULL_POWER_ANGULAR_VELOCITY;

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

    if (controller.getAButton()) {
      drive.fieldOrientedDrive(1, 0.0, 0.0);
    }

    // drive.fieldOrientedDrive(yVelocity, xVelocity, 0.0);
  }

  public double getControllerAxis(int id) {
    XboxController controller = xboxSupplier.get();
    double xValue = controller.getRawAxis(id);
    double yValue = 0;
    if (id == 0 || id == 1) { // Left Stick
      xValue = controller.getRawAxis(0);
      yValue = controller.getRawAxis(1);
    }
    if (id == 2 || id == 3) { // Right Stick
      xValue = controller.getRawAxis(2);
      yValue = controller.getRawAxis(3);
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