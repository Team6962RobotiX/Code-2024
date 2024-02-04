// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import frc.robot.util.LimelightHelpers;
import frc.robot.util.Logging.Logger;
import frc.robot.Constants;
import frc.robot.Constants.LIMELIGHT;
import frc.robot.subsystems.drive.SwerveDrive;


public class ApriltagPose extends SubsystemBase {
  private LimelightHelpers.LimelightResults limelightData;
  private SwerveDrive swerveDrive;

  public ApriltagPose(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void periodic() {
    
  }

  public static Pose2d getRobotPose2d() {
    double[] apriltagResult = NetworkTableInstance.getDefault().getTable(LIMELIGHT.NAME).getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    if (apriltagResult.length < 6) {
      System.err.println("No Pose Data");
      return null;
    }
    Translation2d translation = new Translation2d(apriltagResult[0], apriltagResult[1]);
    Rotation2d rotation = new Rotation2d(Units.degreesToRadians(apriltagResult[5]));
    Pose2d robotPose = new Pose2d(translation, rotation);

    if (robotPose.equals(new Pose2d())) {
      return null;
    }

    return robotPose;
  }

  @Override
  public void simulationPeriodic() {

  }
}