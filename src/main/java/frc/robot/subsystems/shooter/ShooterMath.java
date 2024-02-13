package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SHOOTER.PIVOT;
import frc.robot.Constants.SHOOTER.WHEELS;
import frc.robot.Field;

public class ShooterMath {
  
  /**
   * 
   * @param targetPoint 3D position of target point on the field
   * @param currentPose Current swerve drive pose
   * @param shooterWheelVelocity shooter wheel velocity in rad/s
   * @return
   */
  public static Rotation2d calcPivotAngle(Translation3d targetPoint, Pose2d currentPose, double shooterWheelVelocity) {
    double targetHeight = targetPoint.getZ() - PIVOT.POSITION.getZ();
    double targetDistance = calcShooterLocationOnField(currentPose).toTranslation2d().getDistance(targetPoint.toTranslation2d());
    double projectileVelocity = calcProjectileVelocity(shooterWheelVelocity);
    double gravity = 9.80;
    return Rotation2d.fromRadians(Math.atan((Math.pow(projectileVelocity, 2.0) - Math.sqrt(Math.pow(projectileVelocity, 4.0) - gravity * (gravity * Math.pow(targetDistance, 2.0) + 2.0 * targetHeight * Math.pow(projectileVelocity, 2.0)))) / (gravity * targetDistance)));
  }

  /**
   * 
   * @param shooterWheelVelocity shooter wheel velocity in rad/s
   * @return Projectile exit velocity in m/s
   */
  public static double calcProjectileVelocity(double shooterWheelVelocity) {
    // Derived from https://www.reca.lc/shooterWheel
    double shooterWheelSurfaceSpeed = shooterWheelVelocity * WHEELS.WHEEL_RADIUS;
    double speedTransferPercentage = (WHEELS.TOTAL_MOI * 20.0) / (WHEELS.PROJECTILE_MASS * WHEELS.WHEEL_RADIUS * 2.0 * WHEELS.WHEEL_RADIUS * 2.0 * 7.0 + WHEELS.TOTAL_MOI * 40.0);
    return shooterWheelSurfaceSpeed * speedTransferPercentage;
  }

  
  public static double calcShotChance(Translation3d targetPoint, Pose2d currentPose, Translation2d currentVelocity, Rotation2d measuredPivotAngle, double shooterWheelVelocity) {
    Translation3d shooterLocation = calcShooterLocationOnField(currentPose);
    double floorDistance = targetPoint.toTranslation2d().getDistance(shooterLocation.toTranslation2d());
    double totalDistance = targetPoint.getDistance(shooterLocation);
    
    double speakerVerticalDegreesOfView = Units.radiansToDegrees(2.0 * Math.atan((Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0 / totalDistance));
    double speakerLateralDegreesOfView = Units.radiansToDegrees(2.0 * Math.atan((Field.SPEAKER_WIDTH - Field.NOTE_LENGTH) / 2.0 / totalDistance));
    speakerVerticalDegreesOfView *= Math.cos(Math.PI / 2.0 - Math.atan((Field.SPEAKER.getZ() - PIVOT.POSITION.getZ()) / floorDistance) - Units.degreesToRadians(14.0));
    
    double veritcalDegreesOfAccuracy = PIVOT.ANGLE_PRECISION.getDegrees() * 2.0;
    double lateralDegreesOfAccuracy  = 0.5 * 2.0;
    
    Translation3d aimingPoint = calcVelocityCompensatedPoint(targetPoint, currentPose, currentVelocity, shooterWheelVelocity);
    Rotation2d idealHeading = aimingPoint.toTranslation2d().minus(currentPose.getTranslation()).getAngle();
    Rotation2d idealPivotAngle = calcPivotAngle(aimingPoint, currentPose, shooterWheelVelocity);

    double verticalErrorDegrees = Math.abs(idealPivotAngle.minus(measuredPivotAngle).getDegrees());
    double lateralErrorDegrees = Math.abs(idealHeading.minus(currentPose.getRotation()).getDegrees());

    double verticalValidShotDegrees = Math.min((speakerVerticalDegreesOfView / 2.0 + veritcalDegreesOfAccuracy / 2.0) - verticalErrorDegrees, Math.min(speakerVerticalDegreesOfView, veritcalDegreesOfAccuracy));
    double lateralValidShotDegrees = Math.min((speakerLateralDegreesOfView / 2.0 + lateralDegreesOfAccuracy / 2.0) - lateralErrorDegrees, Math.min(speakerLateralDegreesOfView, lateralDegreesOfAccuracy));
    verticalValidShotDegrees = Math.max(0, verticalValidShotDegrees);
    lateralValidShotDegrees = Math.max(0, lateralValidShotDegrees);

    return (verticalValidShotDegrees * lateralValidShotDegrees) / (veritcalDegreesOfAccuracy * lateralDegreesOfAccuracy);
  }

  
  public static Translation3d calcShooterLocationOnField(Pose2d currentPose) {
    Translation2d swerveDrivePosition = currentPose.getTranslation();
    Translation3d shooterPositionRotated = PIVOT.POSITION.rotateBy(new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians()));
    return new Translation3d(shooterPositionRotated.getX() + swerveDrivePosition.getX(), shooterPositionRotated.getY() + swerveDrivePosition.getY(), shooterPositionRotated.getZ());
  }


  public static Translation3d calcVelocityCompensatedPoint(Translation3d targetPoint, Pose2d currentPose, Translation2d currentVelocity, double shooterWheelVelocity) {
    double distanceFromSpeaker = calcShooterLocationOnField(currentPose).getDistance(targetPoint);

    Translation2d projectileOffset = currentVelocity.times(distanceFromSpeaker / calcProjectileVelocity(shooterWheelVelocity));
    
    return new Translation3d(
      targetPoint.getX() - projectileOffset.getX(),
      targetPoint.getY() - projectileOffset.getY(),
      targetPoint.getZ()
    );
  }

}
