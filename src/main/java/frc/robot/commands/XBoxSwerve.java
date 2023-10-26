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
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.INPUT_MATH;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_MATH;
import frc.robot.subsystems.SwerveController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

/** An example command that uses an example subsystem. */

public class XBoxSwerve extends CommandBase {
  private XboxController controller;
  private final SwerveController swerveController;
  
  public XBoxSwerve(SwerveController swerveController, Supplier<XboxController> xboxSupplier) {
    this.swerveController = swerveController;
    controller = xboxSupplier.get();
    addRequirements(swerveController, swerveController.getSwerveDrive());
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
      swerveController.stopModules();
      return;
    }

    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    Translation2d leftStick = new Translation2d(controller.getLeftX(), controller.getLeftY());
    Translation2d rightStick = new Translation2d(controller.getRightX(), controller.getRightY());
    
    if (RobotBase.isSimulation()) {
      leftTrigger = (controller.getRawAxis(5) + 1.0) / 2.0;
      rightTrigger = (controller.getRawAxis(4) + 1.0) / 2.0;
      leftStick = new Translation2d(controller.getRawAxis(0), controller.getRawAxis(1));
      rightStick = new Translation2d(controller.getRawAxis(2), controller.getRawAxis(3));
    }

    // Deadbands
    leftStick = INPUT_MATH.circular(leftStick, 0.0, Constants.map(leftTrigger, 0, 1, Math.PI / 8.0, Math.PI / 4.0));
    rightStick = INPUT_MATH.circular(rightStick, 0.0, Math.PI / 8);

    
    if (rightTrigger > 0.0) {
      // Angular movement for each trigger
      swerveController.addAngularVelocity((-rightTrigger) * swerveController.MAX_ANGULAR_VELOCITY);
    }

    // Some shenanigans to calculate the right stick angle
    double rightStickAngle = ((-Math.atan2(rightStick.getY(), rightStick.getX()) + Math.PI + Math.PI / 2) % (Math.PI * 2)) - Math.PI;
    double RStickMagnitude = rightStick.getNorm();
    if (RStickMagnitude > 0.8) {
      swerveController.setTargetRobotAngle(rightStickAngle);
    }
    
    // Rotate to nearest 90 degrees when any bumpers are pressed
    if (controller.getLeftBumper() || controller.getRightBumper()) {
      swerveController.snapToNearest90();
    }
        
    // Left stick field oriented drive
    swerveController.addVelocity(swerveController.joystickToFieldMovement(leftStick.times(Constants.map(leftTrigger, 0.0, 1.0, swerveController.NOMINAL_DRIVE_VELOCITY, swerveController.MAX_DRIVE_VELOCITY))));

    // Zero heading when Y is pressed
    if (controller.getYButton()) {
      swerveController.zeroHeading();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveController.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}