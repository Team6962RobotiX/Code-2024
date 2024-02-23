package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.software.Logging.Logger;
import frc.robot.Constants.Field;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.SHOOTER_PIVOT;
import frc.robot.Constants.Constants.SHOOTER_WHEELS;

public class ShooterMath {
  
  /**
   * 
   * @param targetPoint 3D position of target point on the field
   * @param currentPose Current swerve drive pose
   * @param shooterWheelVelocity shooter wheel velocity in rad/s
   * @return
   */
  
  
  // gets the position you should shoot from, using the given shooter angle
  public static double calcShootingDistance(Translation3d targetPoint, double shooterWheelVelocity, Rotation2d pivotAngle){
    // vertical distance between release point and target
    double targetHeight = targetPoint.getZ() - SHOOTER_PIVOT.POSITION.getZ(); 
    double projectileVelocity = calcProjectileVelocity(shooterWheelVelocity);
    double gravity = 9.80;
    double shootingDistance;
    
    try{
      shootingDistance = (
        ((Math.pow(projectileVelocity, 2.0) * Math.tan(pivotAngle.getDegrees())) / gravity) - 
        ((Math.sqrt(-2.0 * gravity * targetHeight * (Math.pow(projectileVelocity, 2.0)) * 
        Math.pow(Math.tan(pivotAngle.getDegrees()), 2.0)) + (Math.pow(projectileVelocity, 4.0) * 
        Math.pow(Math.tan(pivotAngle.getDegrees()), 2.0)) - (2.0 * gravity * targetHeight * 
        Math.pow(projectileVelocity, 2.0))) / 2.0)) / (Math.pow(Math.tan(pivotAngle.getDegrees()), 2.0) + 1.0);
    }
    catch(Exception e){
      return 0.0;
    }

    //todo: maybe return a band of pose2ds
    return shootingDistance;

  }
    
  public static Rotation2d calcPivotAngle(Translation3d targetPoint, Pose2d currentPose, double shooterWheelVelocity) {
    if (shooterWheelVelocity == 0.0) return new Rotation2d(0.0);
    
    Rotation2d pivotAngle = Rotation2d.fromDegrees(0);
    int iterations = 4;

    for (int i = 0; i < iterations; i++) {
      Translation3d shooterLocation = calcShooterLocationOnField(currentPose, pivotAngle);

      Logger.log("shooterLocation", shooterLocation);

      double targetHeight = targetPoint.getZ() - shooterLocation.getZ();
      double floorDistance = shooterLocation.toTranslation2d().getDistance(targetPoint.toTranslation2d());
      double projectileVelocity = calcProjectileVelocity(shooterWheelVelocity);
      double gravity = 9.80;
      Rotation2d exitAngle = Rotation2d.fromRadians(Math.atan((Math.pow(projectileVelocity, 2.0) - Math.sqrt(Math.pow(projectileVelocity, 4.0) - gravity * (gravity * Math.pow(floorDistance, 2.0) + 2.0 * targetHeight * Math.pow(projectileVelocity, 2.0)))) / (gravity * floorDistance)));
      pivotAngle = exitAngle.plus(SHOOTER_PIVOT.NOTE_ROTATION_OFFSET);
    }

    return pivotAngle;
  }

  /**
   * 
   * @param shooterWheelVelocity shooter wheel velocity in rad/s
   * @return Projectile exit velocity in m/s
   */
  public static double calcProjectileVelocity(double shooterWheelVelocity) {
    // Derived from https://www.reca.lc/shooterWheel
    double shooterWheelSurfaceSpeed = shooterWheelVelocity * SHOOTER_WHEELS.WHEEL_RADIUS;
    double speedTransferPercentage = (SHOOTER_WHEELS.TOTAL_MOI * 20.0) / (SHOOTER_WHEELS.PROJECTILE_MASS * SHOOTER_WHEELS.WHEEL_RADIUS * 2.0 * SHOOTER_WHEELS.WHEEL_RADIUS * 2.0 * 7.0 + SHOOTER_WHEELS.TOTAL_MOI * 40.0);
    return shooterWheelSurfaceSpeed * speedTransferPercentage;
  }

  public static double calcShotChance(Translation3d targetPoint, Pose2d currentPose, Translation2d currentVelocity, Rotation2d measuredPivotAngle, double shooterWheelVelocity) {
    Translation3d shooterLocation = calcShooterLocationOnField(currentPose, measuredPivotAngle);
    Translation3d aimingPoint = calcVelocityCompensatedPoint(targetPoint, currentPose, currentVelocity, shooterWheelVelocity, measuredPivotAngle);

    double totalDistance = targetPoint.getDistance(shooterLocation);
    
    Translation3d speakerOffset = new Translation3d(
      Math.acos(Field.SPEAKER_ANGLE) * (Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0,
      0.0,
      Math.asin(Field.SPEAKER_ANGLE) * (Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0
    );

    Translation3d minAimingPoint = aimingPoint.minus(speakerOffset);
    Translation3d maxAimingPoint = aimingPoint.plus(speakerOffset);

    Rotation2d minPivotAngle = calcPivotAngle(minAimingPoint, currentPose, shooterWheelVelocity);
    Rotation2d maxPivotAngle = calcPivotAngle(maxAimingPoint, currentPose, shooterWheelVelocity);

    double speakerVerticalDegreesOfView = maxPivotAngle.minus(minPivotAngle).getDegrees();
    double speakerLateralDegreesOfView = Units.radiansToDegrees(2.0 * Math.atan((Field.SPEAKER_WIDTH - Field.NOTE_LENGTH) / 2.0 / totalDistance));
    
    double veritcalDegreesOfAccuracy = SHOOTER_PIVOT.ANGLE_PRECISION.getDegrees() * 2.0;
    double lateralDegreesOfAccuracy  = 0.5 * 2.0;
    
    Rotation2d idealHeading = aimingPoint.toTranslation2d().minus(currentPose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0));
    Rotation2d idealPivotAngle = calcPivotAngle(aimingPoint, currentPose, shooterWheelVelocity);

    double verticalErrorDegrees = Math.abs(idealPivotAngle.minus(measuredPivotAngle).getDegrees());
    double lateralErrorDegrees = Math.abs(idealHeading.minus(currentPose.getRotation()).getDegrees());

    double verticalValidShotDegrees = Math.min((speakerVerticalDegreesOfView / 2.0 + veritcalDegreesOfAccuracy / 2.0) - verticalErrorDegrees, Math.min(speakerVerticalDegreesOfView, veritcalDegreesOfAccuracy));
    double lateralValidShotDegrees = Math.min((speakerLateralDegreesOfView / 2.0 + lateralDegreesOfAccuracy / 2.0) - lateralErrorDegrees, Math.min(speakerLateralDegreesOfView, lateralDegreesOfAccuracy));
    verticalValidShotDegrees = Math.max(0, verticalValidShotDegrees);
    lateralValidShotDegrees = Math.max(0, lateralValidShotDegrees);
   
    double shotChance = (verticalValidShotDegrees * lateralValidShotDegrees) / (veritcalDegreesOfAccuracy * lateralDegreesOfAccuracy);
    
    // if (idealPivotAngle.getDegrees() < Preferences.SHOOTER_PIVOT.MAX_ANGLE.getDegrees() && idealPivotAngle.getDegrees() > Preferences.SHOOTER_PIVOT.MIN_ANGLE.getDegrees() && shotChance == 1.0) {
    //   List<Pose2d> possibleShotPositions = SwerveDrive.getField().getObject("possibleShotPositions").getPoses();
    //   possibleShotPositions.add(currentPose);
    //   SwerveDrive.getField().getObject("possibleShotPositions").setPoses(possibleShotPositions);
    // }

    return shotChance;
  }

  // Assuming we're already lined up with the target, what's the chance we'll hit it?
  // public static double calcOptimalShotChance(Translation3d targetPoint, Pose2d currentPose, Translation2d currentVelocity) {
  //   double shooterWheelVelocity = Preferences.SHOOTER_WHEELS.TARGET_SPEED;
  //   Translation3d aimingPoint = calcVelocityCompensatedPoint(targetPoint, currentPose, currentVelocity, shooterWheelVelocity, pivotAngle);

  //   currentPose = new Pose2d(
  //     currentPose.getTranslation(),
  //     aimingPoint.toTranslation2d().minus(currentPose.getTranslation()).getAngle()
  //   );

  //   return ShooterMath.calcShotChance(targetPoint, currentPose, currentVelocity, ShooterMath.calcPivotAngle(aimingPoint, currentPose, shooterWheelVelocity), shooterWheelVelocity);
  // }
  
  public static Translation3d calcShooterLocationOnField(Pose2d currentPose, Rotation2d pivotAngle) {
    Translation2d swerveDrivePosition = currentPose.getTranslation();
    Translation3d shooterPosition = SHOOTER_PIVOT.POSITION;
    shooterPosition = shooterPosition.plus(new Translation3d(
      -SHOOTER_PIVOT.SHOOTER_LENGTH * Math.cos(pivotAngle.getRadians()),
      0.0,
      SHOOTER_PIVOT.SHOOTER_LENGTH * Math.sin(pivotAngle.getRadians())
    ));
    Translation3d shooterPositionRotated = shooterPosition.rotateBy(new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians()));
    return new Translation3d(shooterPositionRotated.getX() + swerveDrivePosition.getX(), shooterPositionRotated.getY() + swerveDrivePosition.getY(), shooterPositionRotated.getZ());
  }


  public static Translation3d calcVelocityCompensatedPoint(Translation3d targetPoint, Pose2d currentPose, Translation2d currentVelocity, double shooterWheelVelocity, Rotation2d pivotAngle) {
    if (shooterWheelVelocity == 0.0) return targetPoint;

    double distanceFromSpeaker = calcShooterLocationOnField(currentPose, pivotAngle).getDistance(targetPoint);

    Translation2d projectileOffset = currentVelocity.times(distanceFromSpeaker / calcProjectileVelocity(shooterWheelVelocity));
    
    return new Translation3d(
      targetPoint.getX() - projectileOffset.getX(),
      targetPoint.getY() - projectileOffset.getY(),
      targetPoint.getZ()
    );
  }

}
