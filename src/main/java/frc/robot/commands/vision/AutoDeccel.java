package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE_DRIVE.AUTONOMOUS.DECCEL_CURVE;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Camera;

public class AutoDeccel extends Command {
  private SwerveDrive drive;
  private Camera camera;
  private double speedScalar;

  public AutoDeccel(SwerveDrive drive, Camera camera) {
    this.drive = drive;
    this.camera = camera;
    addRequirements();
  }
  
  @Override
  public void execute() {
    speedScalar = DECCEL_CURVE.getScale(camera.getBestTargetDist());
    drive.setSpeedScale(speedScalar);
  }

  @Override
  public void end(boolean interrupted) {
    drive.setSpeedScale(1.0);
  }
}