// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.vision;

import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Limelight;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class MoveToNote extends Command {
  private final SwerveDrive swerveDrive;
  private final CommandXboxController controller;
  private final String cameraName;
  private Pose2d targetPose;
  private Command goToCommand;

  public MoveToNote(String cameraName, SwerveDrive swerveDrive, CommandXboxController controller) {
    
    this.swerveDrive = swerveDrive;
    this.controller = controller;
    this.cameraName = cameraName;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //The robot won't move unless it sees a note
    targetPose = swerveDrive.getPose();
    goToCommand = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //If the camera can see the note, it updates the position. 
    //As soon at the camera can't see the note, the robot continues driving to the last known note position.
    if (Limelight.targetArea(cameraName) > 0.0) {
      Translation2d translation = swerveDrive.getPose().getTranslation();
      Rotation2d rotation = swerveDrive.getPose().getRotation();

      Rotation2d targetHeading = rotation.minus(Rotation2d.fromDegrees(Limelight.targetHorizontal(cameraName)));

      translation = translation.plus(new Translation2d(
        Limelight.getNoteDist(cameraName),
        0.0
      ).rotateBy(targetHeading));

      targetPose = new Pose2d(translation, targetHeading);

      if (goToCommand == null || goToCommand.isFinished()) {
        goToCommand = swerveDrive.goToSimple(targetPose, () -> true);
        goToCommand.schedule();
      }
    }


















    
    //THIS DOES BOTH ROTATION AND TRANSLATION
    // swerveDrive.goTo(targetPose, controller);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (goToCommand != null) goToCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

