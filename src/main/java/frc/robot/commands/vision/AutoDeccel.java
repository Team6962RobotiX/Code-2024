package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Limelight;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDeccel extends CommandBase {
  private SwerveDrive drive;
  private Limelight limelight;

  public AutoDeccel(SwerveDrive drive, Limelight limelight) {
    this.drive = drive;
    this.limelight = limelight;

    addRequirements();
  }
  
  @Override
  public void execute() {
    // System.out.print("MaxSpeed - ");
    // System.out.println(getMaxSpeed());

    drive.setSpeedCap(getMaxSpeed());

    System.out.print("SpeedCap - ");
    SmartDashboard.putNumber("getLastKnownAprilTagZ", limelight.getLastKnownAprilTagZ());
    System.out.println(drive.getSpeedCap());
  }

  @Override
  public void end(boolean interrupted) {
    drive.setSpeedCap(1);
  }
  
  public double getMaxSpeed() {
    double z = limelight.getLastKnownAprilTagZ();
    double initialDist = 4;
    double initialSpeed = 0.9;
    double finalDist = 1.68;
    double finalSpeed = 0.36;
    double maxSpeed = ((finalSpeed - initialSpeed)/(finalDist - initialDist)) *(z - initialDist) + initialSpeed;
    maxSpeed = Math.max(0.18, maxSpeed); // Add FINE_CONTROL_POWER constant later.
    System.out.println(z);
    return maxSpeed;
  }
}