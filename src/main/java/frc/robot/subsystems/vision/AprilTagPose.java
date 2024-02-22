
package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.subsystems.drive.SwerveDrive;


public class AprilTagPose extends SubsystemBase {
  private static String getLimelightNameWithClosestTarget(String[] names) {
    Map<String, Double> namesWithTags = new HashMap<>();
    for (String name : names) {
      Pose3d pose = Limelight.botposeBlue(name);
      if (pose.equals(new Pose3d())) {
        continue;
      }
      namesWithTags.put(name, Limelight.targetArea(name));
    }
    if (namesWithTags.isEmpty()) {
      return null;
    }
    Entry<String, Double> closestLimelight = Collections.max(namesWithTags.entrySet(), Map.Entry.comparingByValue());
    return closestLimelight.getKey();
  }

  public static void injectVisionData(String[] names, SwerveDrive swerveDrive) {
    String bestLimelightName = getLimelightNameWithClosestTarget(names);
    if (bestLimelightName == null) {
      return;
    }
    swerveDrive.addVisionMeasurement(Limelight.botposeBlue(bestLimelightName).toPose2d(), Timer.getFPGATimestamp() - (Limelight.totalLatency(bestLimelightName) / 1000));
  }
}