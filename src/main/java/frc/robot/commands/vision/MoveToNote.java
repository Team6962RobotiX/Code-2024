// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.vision;

import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Camera;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;

public class MoveToNote extends Command {
  private final SwerveDrive swerveDrive;
  private final Camera camera;
  private final XboxController controller;
  private Pose2d targetPose;

  public MoveToNote(Camera camera, SwerveDrive swerveDrive, XboxController controller) {
    
    this.swerveDrive = swerveDrive;
    this.camera = camera;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camera, swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //The robot won't move unless it sees a note
    targetPose = swerveDrive.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //If the camera can see the note, it updates the position. 
    //As soon at the camera can't see the note, the robot continues driving to the last known note position.
    if (camera.getNoteDist() != 0){
      Translation2d translation = swerveDrive.getPose().getTranslation();
      Rotation2d rotation = swerveDrive.getPose().getRotation();


      double targetX = translation.getX() + camera.getNoteDist()*Math.sin(camera.getTX()*Math.PI/180);
      double targetY = translation.getY() + camera.getNoteDist()*Math.cos(camera.getTX()*Math.PI/180);
      double targetHeading = (rotation.getDegrees() + camera.getTX())*Math.PI/180;

      targetPose = new Pose2d(targetX, targetY, new Rotation2d(targetHeading));
    }

    swerveDrive.goTo(targetPose, controller);

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
