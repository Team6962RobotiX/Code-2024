
package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.Constants.Field;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.software.LimelightHelpers;
import frc.robot.util.software.LimelightHelpers.PoseEstimate;


public class AprilTags extends SubsystemBase {
  public static void injectVisionData(Map<String, Pose3d> cameraPoses, SwerveDrive swerveDrive) {
    if (swerveDrive.getRotationalVelocity() > SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY / 10.0) {
      return;
    }

    List<LimelightHelpers.PoseEstimate> poseEstimates = cameraPoses.keySet().stream().map(LimelightHelpers::getBotPoseEstimate_wpiBlue).collect(Collectors.toList());

    int tagCount = 0;
    for (PoseEstimate poseEstimate : poseEstimates) {
      tagCount += poseEstimate.tagCount;
    }
    if (tagCount == 0) return;

    for (PoseEstimate poseEstimate : poseEstimates) {
      if (tagCount == 1) continue;
      if (poseEstimate.pose.getTranslation().getDistance(swerveDrive.getPose().getTranslation()) > (Field.LENGTH + Field.WIDTH) * 2.0) continue;
      if (poseEstimate.pose.getX() < 0.0 || poseEstimate.pose.getY() < 0.0 || poseEstimate.pose.getX() > Field.LENGTH || poseEstimate.pose.getY() > Field.WIDTH) continue;
      
      double rotationAccuracy = Units.degreesToRadians(999999);
      double translationAccuracy = (swerveDrive.getFieldVelocity().getNorm() + poseEstimate.avgTagDist) / Math.pow(poseEstimate.tagCount, 2.0);
      if (swerveDrive.canZeroHeading() && tagCount >= 2) {
        rotationAccuracy = Units.degreesToRadians(30.0);
      }
      swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(translationAccuracy, translationAccuracy, rotationAccuracy));
      swerveDrive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
      LEDs.setState(LEDs.State.HAS_VISION_TARGET);
    }
  }

  public static void printConfig(Map<String, Pose3d> cameraPoses) {
    System.out.println(
"""


//////////////////////////////////////
///// LIMELIGHT POSITION CONFIG //////
//////////////////////////////////////
"""
    );
    for (Map.Entry<String, Pose3d> limelight : cameraPoses.entrySet()) {
      System.out.println(
        String.format(
          """
          ----- %s.local:5801 -----
            LL Forward: %.5f
            LL Right:   %.5f
            LL Up:      %.5f
            LL Roll:    %.5f
            LL Pitch:   %.5f
            LL Yaw:     %.5f
          """,
          limelight.getKey(),
          limelight.getValue().getTranslation().getX(),
          limelight.getValue().getTranslation().getY(),
          limelight.getValue().getTranslation().getZ(),
          Units.radiansToDegrees(limelight.getValue().getRotation().getX()),
          Units.radiansToDegrees(limelight.getValue().getRotation().getY()),
          Units.radiansToDegrees(limelight.getValue().getRotation().getZ())
        )
      );
    }
  }
}