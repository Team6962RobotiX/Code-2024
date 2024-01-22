// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.util.MathUtils;
import frc.robot.util.MathUtils.InputMath;

public class XBoxSwerve extends Command {
  private XboxController controller;
  private SwerveDrive swerveDrive;
  
  public final double MAX_DRIVE_VELOCITY = SwerveModule.calcWheelVelocity(SWERVE_DRIVE.TELEOPERATED_BOOST_DRIVE_POWER);
  public final double NOMINAL_DRIVE_VELOCITY = SwerveModule.calcWheelVelocity(SWERVE_DRIVE.TELEOPERATED_DRIVE_POWER);
  public final double MAX_ANGULAR_VELOCITY = SwerveDrive.toAngular(SwerveModule.calcWheelVelocity(SWERVE_DRIVE.TELEOPERATED_ROTATE_POWER));

  private double targetRobotAngle = 0.0;
  
  private Translation2d velocity = new Translation2d();
  private double angularVelocity = 0.0;
  
  public XBoxSwerve(SwerveDrive swerveDrive, Supplier<XboxController> xboxSupplier) {
    this.swerveDrive = swerveDrive;
    controller = xboxSupplier.get();
    controller.setRumble(RumbleType.kBothRumble, 1.0);
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

    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    Translation2d leftStick = new Translation2d(controller.getLeftX(), -controller.getLeftY());
    Translation2d rightStick = new Translation2d(controller.getRightX(), -controller.getRightY());
    
    if (RobotBase.isSimulation()) {
      leftTrigger = (controller.getRawAxis(5) + 1.0) / 2.0;
      rightTrigger = (controller.getRawAxis(4) + 1.0) / 2.0;
      leftStick = new Translation2d(-controller.getRawAxis(1), -controller.getRawAxis(0));
      rightStick = new Translation2d(-controller.getRawAxis(3), -controller.getRawAxis(2));
    }

    // Deadbands
    leftStick = InputMath.circular(leftStick, 0.0, MathUtils.map(Math.max(leftTrigger, rightTrigger), 0, 1, Math.PI / 16.0, Math.PI / 8.0));
    
    angularVelocity += -rightStick.getX() * MAX_ANGULAR_VELOCITY;
    
    // Rotate to nearest 90 degrees when any bumpers are pressed
    if (controller.getLeftBumper() || controller.getRightBumper()) {
      swerveDrive.setTargetHeading(Rotation2d.fromRadians(Math.round(targetRobotAngle / (Math.PI / 2.0)) * (Math.PI / 2.0)));
    }

    velocity = velocity.plus(leftStick.times(MathUtils.map(Math.max(leftTrigger, rightTrigger), 0, 1, NOMINAL_DRIVE_VELOCITY, MAX_DRIVE_VELOCITY)));

    // Zero heading when Y is pressed
    if (controller.getYButton()) {
      swerveDrive.setHeading(new Rotation2d());
    }

    if (SwerveDrive.toLinear(Math.abs(angularVelocity)) < SWERVE_DRIVE.VELOCITY_DEADBAND) {
      angularVelocity = 0.0;
    }
    if (velocity.getNorm() < SWERVE_DRIVE.VELOCITY_DEADBAND) {
      velocity = new Translation2d();
    }

    
    boolean moving = false;
    moving = moving || SwerveDrive.toLinear(Math.abs(angularVelocity)) > SWERVE_DRIVE.VELOCITY_DEADBAND;
    moving = moving || velocity.getNorm() > SWERVE_DRIVE.VELOCITY_DEADBAND;
    for (SwerveModuleState moduleState : swerveDrive.getTargetModuleStates()) if (Math.abs(moduleState.speedMetersPerSecond) > SWERVE_DRIVE.VELOCITY_DEADBAND) moving = true;
    for (SwerveModuleState moduleState : swerveDrive.getMeasuredModuleStates()) if (Math.abs(moduleState.speedMetersPerSecond) > SWERVE_DRIVE.VELOCITY_DEADBAND) moving = true;
    if (!moving) {
      swerveDrive.parkModules();
      return;
    }

    swerveDrive.driveFieldRelative(velocity.getX(), velocity.getY(), angularVelocity);
    
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