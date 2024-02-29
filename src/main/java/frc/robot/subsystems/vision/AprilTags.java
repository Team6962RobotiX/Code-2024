
package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.software.LimelightHelpers;
import frc.robot.util.software.LimelightHelpers.LimelightResults;
import frc.robot.util.software.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.software.LimelightHelpers.Results;


public class AprilTags extends SubsystemBase {
  public static void injectVisionData(Map<String, Pose3d> cameraPoses, SwerveDrive swerveDrive) {
    for (String name : cameraPoses.keySet()) {
      LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      if (poseEstimate.tagCount >= 1) {
        // Pose3d poseEstimate3d = new Pose3d(poseEstimate.pose);
        // Pose3d robotPose = poseEstimate3d.relativeTo(cameraPoses.get(name));
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(30.0)));
        swerveDrive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
      }
    }
  }
}