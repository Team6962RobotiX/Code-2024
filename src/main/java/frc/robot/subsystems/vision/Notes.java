package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Field;
import frc.robot.util.software.LimelightHelpers;
import frc.robot.util.software.LimelightHelpers.LimelightTarget_Detector;

public class Notes {
  public static List<Translation2d> getNotePositions(String name, Rotation2d pitch, Pose2d robotPose, Translation2d fieldVelocity, Translation3d cameraToRobot) {
    LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(name);
    LimelightTarget_Detector[] targets = results.targetingResults.targets_Detector;

    List<Translation2d> notePositions = new ArrayList<>();
    
    for (LimelightTarget_Detector target : targets) {
      if (target.confidence < 0.8) continue;
      double x = target.tx;
      double y = target.ty;
      double latency = (results.targetingResults.latency_capture + results.targetingResults.latency_jsonParse + results.targetingResults.latency_pipeline) / 1000.0;
      double dist = ((cameraToRobot.getZ() - Field.NOTE_THICKNESS / 2.0) / (Math.tan(Math.abs(Units.degreesToRadians(y) + pitch.getRadians())) * Math.abs(Math.cos(Units.degreesToRadians(x)))));
      Translation2d robotPosition = robotPose.getTranslation().minus(fieldVelocity.times(latency / 1000.0));
      Translation2d notePosition = robotPosition.plus(cameraToRobot.toTranslation2d().plus(new Translation2d(dist, 0)).rotateBy(robotPose.getRotation()));
      notePositions.add(notePosition);
    }

    return notePositions;
  }
}
