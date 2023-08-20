// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** An example command that uses an example subsystem. */


public class JoystickSwerve extends CommandBase {
  private final Drive drive;
  private final Supplier<Joystick> joystickSupplier;

  ChassisSpeeds speeds;
  Translation2d m_frontLeftLocation = new Translation2d(12.375, 12.75);
  Translation2d m_frontRightLocation = new Translation2d(12.375, -12.75);
  Translation2d m_backLeftLocation = new Translation2d(-12.375, 12.75);
  Translation2d m_backRightLocation = new Translation2d(-12.375, -12.75);
  

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  // Convert to module states
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

  // Front left module state
  SwerveModuleState frontLeft = moduleStates[0];

  // Front right module state
  SwerveModuleState frontRight = moduleStates[1];

  // Back left module state
  SwerveModuleState backLeft = moduleStates[2];

  // Back right module state
  SwerveModuleState backRight = moduleStates[3];

  public JoystickSwerve(Drive drive, Supplier<Joystick> joystickSupplier) {
    this.drive = drive;
    this.joystickSupplier = joystickSupplier;
    
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
    Joystick joystick = joystickSupplier.get();
    this.speeds = new ChassisSpeeds(joystick.getY(), joystick.getX(), joystick.getTwist());
    double speed = joystick.getRawAxis(1) / 1.0;

    if (joystick.getTrigger()) {
      drive.runSpark(speed);
    } else {
      drive.runSpark(0.0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
