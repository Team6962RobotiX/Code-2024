package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.SHOOTER_PIVOT;
import frc.robot.Constants.Constants.SHOOTER_WHEELS;
import frc.robot.Constants.Field;
import frc.robot.Constants.Preferences;

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
    
    try {
      shootingDistance = (
        ((Math.pow(projectileVelocity, 2.0) * Math.tan(pivotAngle.getDegrees())) / gravity) - 
        ((Math.sqrt(-2.0 * gravity * targetHeight * (Math.pow(projectileVelocity, 2.0)) * 
        Math.pow(Math.tan(pivotAngle.getDegrees()), 2.0)) + (Math.pow(projectileVelocity, 4.0) * 
        Math.pow(Math.tan(pivotAngle.getDegrees()), 2.0)) - (2.0 * gravity * targetHeight * 
        Math.pow(projectileVelocity, 2.0))) / 2.0)) / (Math.pow(Math.tan(pivotAngle.getDegrees()), 2.0) + 1.0);
    } catch (Exception e){
      return 0.0;
    }

    //todo: maybe return a band of pose2ds
    return shootingDistance;

  }

  public static boolean inRange(Translation3d targetPoint, Pose2d currentPose, double shooterWheelVelocity) {
    Rotation2d pivotAngle = calcPivotAngle(targetPoint, currentPose, shooterWheelVelocity);
    return pivotAngle.getRadians() != 0 && pivotAngle.getRadians() > Preferences.SHOOTER_PIVOT.MIN_ANGLE.getRadians() && pivotAngle.getRadians() < Preferences.SHOOTER_PIVOT.MAX_ANGLE.getRadians();
  }
    
  public static Rotation2d calcPivotAngle(Translation3d targetPoint, Pose2d currentPose, double shooterWheelVelocity) {
    if (shooterWheelVelocity == 0.0) return new Rotation2d(0.0);
    
    Rotation2d pivotAngle = Rotation2d.fromDegrees(0);
    int iterations = 4;

    for (int i = 0; i < iterations; i++) {
      Translation3d shooterLocation = calcShooterLocationOnField(currentPose, pivotAngle);

      double targetHeight = targetPoint.getZ() - shooterLocation.getZ();
      double floorDistance = shooterLocation.toTranslation2d().getDistance(targetPoint.toTranslation2d());
      double projectileVelocity = calcProjectileVelocity(shooterWheelVelocity);
      double gravity = 9.80;

      double exitRadians = Math.atan((Math.pow(projectileVelocity, 2.0) - Math.sqrt(Math.pow(projectileVelocity, 4.0) - gravity * (gravity * Math.pow(floorDistance, 2.0) + 2.0 * targetHeight * Math.pow(projectileVelocity, 2.0)))) / (gravity * floorDistance));
      double distanceAtApex = projectileVelocity * Math.cos(exitRadians) * (projectileVelocity * (Math.sin(exitRadians) / gravity));
      if (Double.isNaN(exitRadians) || distanceAtApex < floorDistance) {
        return Rotation2d.fromDegrees(0.0);
      }
      Rotation2d exitAngle = Rotation2d.fromRadians(exitRadians);
      pivotAngle = exitAngle.minus(SHOOTER_PIVOT.NOTE_ROTATION_OFFSET);
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
    // return 1000000.0;
    return 13 * (shooterWheelVelocity / 1000);
    
    // return (shooterWheelVelocity * (SHOOTER_WHEELS.WHEEL_RADIUS + (Field.NOTE_THICKNESS - SHOOTER_WHEELS.COMPRESSION) / 2.0)) / 4.0;
    // double shooterWheelSurfaceSpeed = shooterWheelVelocity * SHOOTER_WHEELS.WHEEL_RADIUS;

    // double speedTransferPercentage = (SHOOTER_WHEELS.TOTAL_MOI * 20.0) / (SHOOTER_WHEELS.PROJECTILE_MASS * SHOOTER_WHEELS.WHEEL_RADIUS * 2.0 * SHOOTER_WHEELS.WHEEL_RADIUS * 2.0 * 7.0 + SHOOTER_WHEELS.TOTAL_MOI * 40.0);
    // return shooterWheelSurfaceSpeed * speedTransferPercentage;
  }

  public static double calcShotChance(Translation3d targetPoint, Pose2d currentPose, Translation2d currentVelocity, Rotation2d measuredPivotAngle, double shooterWheelVelocity) {
    Translation3d shooterLocation = calcShooterLocationOnField(currentPose, measuredPivotAngle);
    Translation3d aimingPoint = calcVelocityCompensatedPoint(targetPoint, currentPose, currentVelocity, shooterWheelVelocity, measuredPivotAngle);

    double totalDistance = targetPoint.getDistance(shooterLocation);
    
    Translation3d speakerOffset = new Translation3d();

    if (Constants.IS_BLUE_TEAM.get()) {
      speakerOffset = new Translation3d(
        Math.cos(Field.SPEAKER_ANGLE) * (Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0,
        0.0,
        Math.sin(Field.SPEAKER_ANGLE) * (Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0
      );
    } else {
      speakerOffset = new Translation3d(
        -Math.cos(-Field.SPEAKER_ANGLE) * (Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0,
        0.0,
        -Math.sin(-Field.SPEAKER_ANGLE) * (Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0
      );
    }

    Translation3d minAimingPoint = aimingPoint.minus(speakerOffset);
    Translation3d maxAimingPoint = aimingPoint.plus(speakerOffset);

    Rotation2d minPivotAngle = calcPivotAngle(minAimingPoint, currentPose, shooterWheelVelocity);
    Rotation2d maxPivotAngle = calcPivotAngle(maxAimingPoint, currentPose, shooterWheelVelocity);

    double speakerVerticalDegreesOfView = maxPivotAngle.minus(minPivotAngle).getDegrees();
    double speakerLateralDegreesOfView = Units.radiansToDegrees(2.0 * Math.atan((Field.SPEAKER_WIDTH - Field.NOTE_LENGTH) / 2.0 / totalDistance));
    
    double velocityErrorDegrees = Math.abs(calcPivotAngle(aimingPoint, currentPose, shooterWheelVelocity + SHOOTER_WHEELS.SPEED_PRECISION).minus(calcPivotAngle(aimingPoint, currentPose, shooterWheelVelocity - SHOOTER_WHEELS.SPEED_PRECISION)).getDegrees());
    
    double verticalDegreesOfAccuracy = SHOOTER_PIVOT.ANGLE_PRECISION.getDegrees() * 2.0 + velocityErrorDegrees;
    double lateralDegreesOfAccuracy  = SHOOTER_PIVOT.HEADING_PRECISION.getDegrees() * 2.0;
    
    Rotation2d idealHeading = aimingPoint.toTranslation2d().minus(currentPose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0));

    Rotation2d idealPivotAngle = calcPivotAngle(aimingPoint, currentPose, shooterWheelVelocity);
    
    double verticalErrorDegrees = Math.abs(idealPivotAngle.minus(measuredPivotAngle).getDegrees());
    double lateralErrorDegrees = Math.abs(idealHeading.minus(currentPose.getRotation()).getDegrees());
    
    double verticalValidShotDegrees = Math.min((speakerVerticalDegreesOfView / 2.0 + verticalDegreesOfAccuracy / 2.0) - verticalErrorDegrees, Math.min(speakerVerticalDegreesOfView, verticalDegreesOfAccuracy));
    double lateralValidShotDegrees = Math.min((speakerLateralDegreesOfView / 2.0 + lateralDegreesOfAccuracy / 2.0) - lateralErrorDegrees, Math.min(speakerLateralDegreesOfView, lateralDegreesOfAccuracy));
    verticalValidShotDegrees = Math.max(0, verticalValidShotDegrees);
    lateralValidShotDegrees = Math.max(0, lateralValidShotDegrees);
   
    double shotChance = (verticalValidShotDegrees * lateralValidShotDegrees) / (verticalDegreesOfAccuracy * lateralDegreesOfAccuracy);
    
    // if (idealPivotAngle.getDegrees() < Preferences.SHOOTER_PIVOT.MAX_ANGLE.getDegrees() && idealPivotAngle.getDegrees() > Preferences.SHOOTER_PIVOT.MIN_ANGLE.getDegrees() && shotChance == 1.0) {
    //   List<Pose2d> possibleShotPositions = SwerveDrive.getField().getObject("possibleShotPositions").getPoses();
    //   possibleShotPositions.add(currentPose);
    //   SwerveDrive.getField().getObject("possibleShotPositions").setPoses(possibleShotPositions);
    // }

    return shotChance;
  }

  // Assuming we're already lined up with the target, what's the chance we'll hit it?
  public static double calcOptimalShotChance(Translation3d targetPoint, Pose2d currentPose) {
    double shooterWheelVelocity = Preferences.SHOOTER_WHEELS.TARGET_SPEED;
    Translation3d aimingPoint = Field.SPEAKER.get();
    
    currentPose = new Pose2d(
      currentPose.getTranslation(),
      aimingPoint.toTranslation2d().minus(currentPose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0))
    );

    return ShooterMath.calcShotChance(targetPoint, currentPose, new Translation2d(), ShooterMath.calcPivotAngle(aimingPoint, currentPose, shooterWheelVelocity), shooterWheelVelocity);
  }
  
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


  public static double calculateFlightTime(Translation3d targetPoint, Pose2d currentPose, double shooterWheelVelocity, Rotation2d pivotAngle) {
    // (v * sin(a) - sqrt(v^2 * sin(a)^2 - 2 * g * h)) / g
    double targetHeight = targetPoint.getZ() - calcShooterLocationOnField(currentPose, pivotAngle).getZ();
    double projectileVelocity = calcProjectileVelocity(shooterWheelVelocity);
    double gravity = 9.80;
    Rotation2d exitAngle = pivotAngle.plus(SHOOTER_PIVOT.NOTE_ROTATION_OFFSET);
    return (projectileVelocity * Math.sin(exitAngle.getRadians()) - Math.sqrt(Math.pow(projectileVelocity * Math.sin(exitAngle.getRadians()), 2.0) - 2.0 * gravity * -targetHeight)) / gravity;
  }

  public static Translation3d calcVelocityCompensatedPoint(Translation3d targetPoint, Pose2d currentPose, Translation2d currentVelocity, double shooterWheelVelocity, Rotation2d pivotAngle) {    
    if (RobotState.isAutonomous()) {
      return targetPoint;
    }
    
    if (shooterWheelVelocity == 0.0) return targetPoint;
    
    double flightTime = calculateFlightTime(targetPoint, currentPose, shooterWheelVelocity, pivotAngle);

    if (Double.isNaN(flightTime)) return targetPoint;
    
    Translation2d projectileOffset = currentVelocity.times(flightTime);
    
    return new Translation3d(
      targetPoint.getX() - projectileOffset.getX(),
      targetPoint.getY() - projectileOffset.getY(),
      targetPoint.getZ()
    );
  }

}
