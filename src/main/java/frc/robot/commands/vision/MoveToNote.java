// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Notes;


public class MoveToNote extends Command {
  private final SwerveDrive swerveDrive;
  private final RobotStateController stateController;
  private final String cameraName;
  private Command goToCommand = Commands.runOnce(() -> {});
  private Translation2d targetingNote;

  public MoveToNote(String cameraName, SwerveDrive swerveDrive, RobotStateController stateController) {
    
    this.swerveDrive = swerveDrive;
    this.cameraName = cameraName;
    this.stateController = stateController;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The robot won't move unless it sees a note
    goToCommand = Commands.runOnce(() -> {});
    goToCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    // If the camera can see the note, it updates the position. 
    // As soon at the camera can't see the note, the robot continues driving to the last known note position.
    Translation2d notePosition = Notes.getNotePosition(cameraName, LIMELIGHT.NOTE_CAMERA_PITCH, swerveDrive, swerveDrive.getFieldVelocity(), LIMELIGHT.NOTE_CAMERA_POSITION);
    if (notePosition != null) {
      swerveDrive.facePoint(() -> notePosition, new Rotation2d());
    }

    // THIS DOES BOTH ROTATION AND TRANSLATION
    // swerveDrive.goTo(targetPose, controller);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    goToCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateController.hasNote();
  }
}

