// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class Limelight extends SubsystemBase {
  private LimelightHelpers.LimelightResults limelightData;
  private String name;
  private ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");
  private double lastKnownAprilTagZ = 0.0;

  public Limelight(String name) {
    this.name = name;
    dashboard.addCamera(name, name, "http://" + name + ".local:5800");
  }

  @Override
  public void periodic() {
    
    limelightData = LimelightHelpers.getLatestResults(name);
    Pose3d targetSpaceTop = LimelightHelpers.getCameraPose3d_TargetSpace("top"); // change later
    Pose3d targetSpaceBottom = LimelightHelpers.getCameraPose3d_TargetSpace("bottom");

    double z_top = Math.abs(targetSpaceTop.getZ());
    double z_bottom = Math.abs(targetSpaceBottom.getZ());

    if (z_top != 0 && z_bottom != 0) {
      lastKnownAprilTagZ = (z_top+z_bottom)/2;
    }else if (z_top != 0){
      lastKnownAprilTagZ = z_top;
    }else if (z_bottom != 0){
      lastKnownAprilTagZ = z_bottom;
    }
  }

  public double getLastKnownAprilTagZ() {
    return lastKnownAprilTagZ;
  }

  public LimelightHelpers.Results getTargetingResults() {
    return limelightData.targetingResults;
  }

  public String getName() {
    return name;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}