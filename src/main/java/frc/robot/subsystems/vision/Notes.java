package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Field;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.software.LimelightHelpers;
import frc.robot.util.software.LimelightHelpers.LimelightTarget_Detector;

public class Notes {
  public static List<Translation2d> getNotePositions(String name, Rotation2d pitch, Pose2d robotPose, Translation2d fieldVelocity, Translation3d cameraToRobot) {
    LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(name);
    LimelightTarget_Detector[] targets = results.targetingResults.targets_Detector;

    List<Translation2d> notePositions = new ArrayList<>();
    for (LimelightTarget_Detector target : targets) {
      double x = target.tx;
      double y = target.ty;
      
      if (target.confidence < 0.8) continue;
      if (Units.degreesToRadians(y) + pitch.getRadians() > 0) continue;

      double latency = (results.targetingResults.latency_capture + results.targetingResults.latency_jsonParse + results.targetingResults.latency_pipeline) / 1000.0;
      double distance = cameraToRobot.getZ() / Math.tan(Units.degreesToRadians(y) + pitch.getRadians());
     
      Translation2d relativePosition = new Translation2d(
        distance * Math.sin(Units.degreesToRadians(x)),
        distance * Math.cos(Units.degreesToRadians(x))
      );
      
      Translation2d robotPosition = robotPose.getTranslation().minus(fieldVelocity.times(latency / 1000.0));
      Translation2d notePosition = robotPosition.plus(relativePosition.rotateBy(robotPose.getRotation()));
      notePositions.add(notePosition);
    }
    
    return notePositions;
  }
}
