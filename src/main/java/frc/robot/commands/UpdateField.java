// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.PhotonLib;

/** An example command that uses an example subsystem. */
public class UpdateField extends Command {
  private final SwerveDrive s_drive;
  private final PhotonLib p_lib;

  public UpdateField(SwerveDrive s_drive, PhotonLib p_lib) {
    this.s_drive = s_drive;
    this.p_lib = p_lib;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p_lib);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    p_lib.UpdateField(s_drive.getPose());
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
