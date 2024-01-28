package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Camera;
import frc.robot.util.Logging.Logger;

public class AutoDeccel extends Command {
  private SwerveDrive drive;
  private Camera camera;
  private double speedScalar;

  public AutoDeccel(SwerveDrive drive, Camera camera) {
    this.drive = drive;
    this.camera = camera;
    //addRequirements(drive, camera);
    addRequirements();
  }
  
  @Override
  public void execute() {
    drive.setAutoSpeedScale(getSpeedScalar());
    Logger.log("vision/scalar", getSpeedScalar());
  }

  @Override
  public void end(boolean interrupted) {
    drive.setAutoSpeedScale(1.0);
  }
  
  private double getSpeedScalar() {
    double currentDist = camera.getBestTargetDist();
    double initialDist = 4;
    double initialSpeed = 0.9;
    double finalDist = 1;
    double finalSpeed = 0;
    speedScalar = ((finalSpeed - initialSpeed)/(finalDist - initialDist)) * (currentDist - initialDist) + initialSpeed;
    return speedScalar;
  }
}