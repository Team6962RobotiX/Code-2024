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

/** An example command that uses an example subsystem. */
public class JoystickSwerve extends CommandBase {
  private final Drive drive;
  private final Supplier<Joystick> joystickSupplier;

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
