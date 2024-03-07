package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Field;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.software.LimelightHelpers;
import frc.robot.util.software.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.util.software.Logging.Logger;

public class Notes {
  public static List<Translation2d> getNotePositions(String name, Rotation2d pitch, SwerveDrive swerveDrive, Translation2d fieldVelocity, Translation3d cameraToRobot) {
    LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(name);
    LimelightTarget_Detector[] targets = results.targetingResults.targets_Detector;

    List<Translation2d> notePositions = new ArrayList<>();
    for (LimelightTarget_Detector target : targets) {
      double x = target.tx;
      double y = target.ty;// - Math.sqrt(Constants.LIMELIGHT.FOV_HEIGHT.getDegrees() * Constants.LIMELIGHT.FOV_HEIGHT.getDegrees() * target.ta)/2;
      
      if (target.confidence < 0.5) continue;
      if (Units.degreesToRadians(y) + pitch.getRadians() > 0) continue;
      
      double latency = (results.targetingResults.latency_capture + results.targetingResults.latency_jsonParse + results.targetingResults.latency_pipeline) / 1000.0;
      double distance = (cameraToRobot.getZ() - Field.NOTE_THICKNESS/2) / -Math.tan(Units.degreesToRadians(y) + pitch.getRadians());
      Logger.log("note-distance", distance);
      Translation2d relativePosition = new Translation2d(
        distance * Math.cos(Units.degreesToRadians(x)),
        -distance * Math.sin(Units.degreesToRadians(x))
      );
      
      Pose2d robotPosition = swerveDrive.getPose(Timer.getFPGATimestamp() - latency);
      Translation2d notePosition = robotPosition.getTranslation().plus(relativePosition.rotateBy(robotPosition.getRotation()));
      notePositions.add(notePosition);
    }
    
    return notePositions;
  }
}
