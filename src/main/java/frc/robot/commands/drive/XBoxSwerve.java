// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import javax.xml.namespace.QName;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.INPUT_MATH;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModule;

/** An example command that uses an example subsystem. */

public class XBoxSwerve extends CommandBase {
  private XboxController controller;
  private SwerveDrive swerveDrive;
  
  public final double MAX_DRIVE_VELOCITY = SwerveModule.calcDriveVelocity(SWERVE_DRIVE.TELEOPERATED_BOOST_DRIVE_POWER);
  public final double NOMINAL_DRIVE_VELOCITY = SwerveModule.calcDriveVelocity(SWERVE_DRIVE.TELEOPERATED_DRIVE_POWER);
  public final double SLOW_DRIVE_VELOCITY = SwerveModule.calcDriveVelocity(SWERVE_DRIVE.TELEOPERATED_SLOW_DRIVE_POWER);
  public final double MAX_ANGULAR_VELOCITY = SwerveDrive.wheelVelocityToRotationalVelocity(SwerveModule.calcDriveVelocity(SWERVE_DRIVE.TELEOPERATED_ROTATE_POWER));

  private double targetRobotAngle = 0.0;
  
  private Translation2d velocity = new Translation2d();
  private double angularVelocity = 0.0;
  
  public XBoxSwerve(SwerveDrive swerveDrive, Supplier<XboxController> xboxSupplier) {
    this.swerveDrive = swerveDrive;
    controller = xboxSupplier.get();
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.setRumble(RumbleType.kBothRumble, 0.000001);
    // Disable drive if the controller disconnects
    if (!controller.isConnected()) {
      swerveDrive.stopModules();
      return;
    }

    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    Translation2d leftStick = new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    Translation2d rightStick = new Translation2d(-controller.getRightY(), -controller.getRightX());
    
    if (RobotBase.isSimulation()) {
      leftTrigger = (controller.getRawAxis(5) + 1.0) / 2.0;
      rightTrigger = (controller.getRawAxis(4) + 1.0) / 2.0;
      leftStick = new Translation2d(-controller.getRawAxis(1), -controller.getRawAxis(0));
      rightStick = new Translation2d(-controller.getRawAxis(3), -controller.getRawAxis(2));
    }

    // Deadbands
    leftStick = INPUT_MATH.circular(leftStick, 0.0, Math.PI / 8.0);
    rightStick = INPUT_MATH.circular(rightStick, 0.0, Math.PI / 8.0);
    
    angularVelocity += rightStick.getY() * MAX_ANGULAR_VELOCITY;
    
    // Rotate to nearest 90 degrees when any bumpers are pressed
    if (controller.getLeftBumper() || controller.getRightBumper()) {
      swerveDrive.setTargetHeading(Rotation2d.fromRadians(Math.round(targetRobotAngle / (Math.PI / 2)) * (Math.PI / 2)));
    }

    velocity = velocity.plus(leftStick.times(Constants.map(Math.max(leftTrigger, rightTrigger), 0, 1, NOMINAL_DRIVE_VELOCITY, MAX_DRIVE_VELOCITY)));

    // Zero heading when Y is pressed
    if (controller.getYButton()) {
      swerveDrive.zeroHeading();
    }

    if (SwerveDrive.rotationalVelocityToWheelVelocity(Math.abs(angularVelocity)) < SWERVE_DRIVE.VELOCITY_DEADBAND) {
      angularVelocity = 0.0;
    }
    if (velocity.getNorm() < SWERVE_DRIVE.VELOCITY_DEADBAND) {
      velocity = new Translation2d();
    }

    
    boolean moving = false;
    moving = moving || SwerveDrive.rotationalVelocityToWheelVelocity(Math.abs(angularVelocity)) > SWERVE_DRIVE.VELOCITY_DEADBAND;
    moving = moving || velocity.getNorm() > SWERVE_DRIVE.VELOCITY_DEADBAND;
    for (SwerveModuleState moduleState : swerveDrive.getTargetModuleStates()) if (Math.abs(moduleState.speedMetersPerSecond) > SWERVE_DRIVE.VELOCITY_DEADBAND) moving = true;
    for (SwerveModuleState moduleState : swerveDrive.getMeasuredModuleStates()) if (Math.abs(moduleState.speedMetersPerSecond) > SWERVE_DRIVE.VELOCITY_DEADBAND) moving = true;
    if (!moving) {
      swerveDrive.parkModules();
      return;
    }

    swerveDrive.fieldOrientedDrive(velocity.getX(), velocity.getY(), angularVelocity);
    
    angularVelocity = 0.0;
    velocity = new Translation2d();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}