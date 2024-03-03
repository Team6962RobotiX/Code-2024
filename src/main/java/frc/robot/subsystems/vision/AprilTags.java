
package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import com.badlogic.gdx.math.MathUtils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.software.LimelightHelpers;
import frc.robot.util.software.LimelightHelpers.LimelightResults;
import frc.robot.util.software.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.software.LimelightHelpers.Results;


public class AprilTags extends SubsystemBase {
  public static void injectVisionData(Map<String, Pose3d> cameraPoses, SwerveDrive swerveDrive) {
    if (swerveDrive.getRotationalVelocity() > SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY / 10.0) {
      return;
    }

    for (String name : cameraPoses.keySet()) {
      LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      if (poseEstimate.tagCount == 0) continue;
      if (RobotState.isAutonomous() && poseEstimate.tagCount == 1) continue;
      double rotationAccuracy = Units.degreesToRadians(999999);
      double translationAccuracy = (swerveDrive.getFieldVelocity().getNorm() + Math.sqrt(poseEstimate.avgTagDist)) / Math.pow(poseEstimate.tagCount, 2.0);
      if (swerveDrive.canZeroHeading() && poseEstimate.tagCount >= 2) {
        rotationAccuracy = Units.degreesToRadians(30.0);
      }
      swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(translationAccuracy, translationAccuracy, rotationAccuracy));
      swerveDrive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
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