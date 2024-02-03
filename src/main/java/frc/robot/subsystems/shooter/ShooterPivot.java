// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;
import frc.robot.Constants;
import frc.robot.Field;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.SHOOTER;
import frc.robot.Constants.SHOOTER.SHOOTER_PIVOT;
import frc.robot.Constants.SHOOTER.SHOOTER_WHEELS;
import frc.robot.Constants.SWERVE_DRIVE;

public class ShooterPivot extends SubsystemBase {
  private Rotation2d targetPivotAngle = new Rotation2d();
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private boolean isCalibrating = false;
  private ShooterWheels shooterWheels;
  private SwerveDrive swerveDrive;
  private double headingVelocity = 0.0;
  private Rotation2d targetHeading = new Rotation2d();
  private Mechanism2d mechanism = new Mechanism2d(0, 0);
  private MechanismRoot2d pivotMechanism = mechanism.getRoot("pivot", SHOOTER_PIVOT.POSITION.getX(), SHOOTER_PIVOT.POSITION.getZ());
  private MechanismLigament2d shooterMechanism = pivotMechanism.append(new MechanismLigament2d("shooter", SHOOTER_PIVOT.LENGTH, 90));

  public ShooterPivot(ShooterWheels shooterWheels, SwerveDrive swerveDrive) {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    motor = new CANSparkMax(CAN.SHOOTER_PIVOT, MotorType.kBrushless);
    encoder = motor.getEncoder();
    encoder.setVelocityConversionFactor(SHOOTER_PIVOT.GEARBOX_REDUCTION);

    String logPath = "flywheel/";
    Logger.autoLog(logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> motor.getMotorTemperature());
    Logger.autoLog(logPath + "position",                () -> encoder.getPosition());
    Logger.autoLog(logPath + "velocity",                () -> encoder.getVelocity());

    StatusChecks.addCheck("Shooter Pivot Motor", () -> motor.getFaults() == 0);

    this.shooterWheels = shooterWheels;
    this.swerveDrive = swerveDrive;

    SmartDashboard.putData("ShooterMechanism", mechanism);
  }

  public void setTargetAngle(Rotation2d angle) {
    targetPivotAngle = angle;
  }

  public Rotation2d getPivotAngle() {
    return targetPivotAngle;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    if (isCalibrating) return;
    
    // Calculate point to aim towards, accounting for current velocity

    Translation3d speakerPosition = Field.SPEAKER_RED;
    Translation3d pointToAimTo = calculateAimingPoint(speakerPosition);
    double floorDistance = speakerPosition.toTranslation2d().getDistance(getShooterLocationOnField(swerveDrive.getPose()).toTranslation2d());
    
    if (floorDistance < 8.0) {
      orientToPoint(pointToAimTo);
    }

    Logger.autoLog("pointToAimTo", () -> pointToAimTo);

    setTargetAngle(shooterWheels.calculatePivotAngle(pointToAimTo));
    shooterWheels.setTargetAngularVelocity(SHOOTER_WHEELS.TARGET_SPEED);

    double shotChance = calculateShotChance(speakerPosition);
    System.out.println("shotChance " + shotChance * 100);

    shooterMechanism.setAngle(getPivotAngle());
    System.out.println(targetPivotAngle);
    // pivotMechanism.setPosition(floorDistance, shotChance);
  }

  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  }

  public Translation3d calculateAimingPoint(Translation3d targetPoint) {
    double distanceFromSpeaker = getShooterLocationOnField(swerveDrive.getPose()).getDistance(targetPoint);

    Translation2d currentVelocity = swerveDrive.getFieldVelocity();
    Translation2d projectileOffset = currentVelocity.times(distanceFromSpeaker / shooterWheels.getProjectileVelocity());
    
    return new Translation3d(
      targetPoint.getX() - projectileOffset.getX(),
      targetPoint.getY() - projectileOffset.getY(),
      targetPoint.getZ()
    );
  }

  public static Translation3d getShooterLocationOnField(Pose2d currentPose) {
    Translation2d swerveDrivePosition = currentPose.getTranslation();
    Translation3d shooterPositionRotated = SHOOTER_PIVOT.POSITION.rotateBy(new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians()));
    return new Translation3d(shooterPositionRotated.getX() + swerveDrivePosition.getX(), shooterPositionRotated.getY() + swerveDrivePosition.getY(), shooterPositionRotated.getZ());
  }

  public void orientToPoint(Translation3d pointToAimTo) {
    Rotation2d newTargetHeading = pointToAimTo.toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle();
    headingVelocity = newTargetHeading.minus(targetHeading).getRadians() / 0.02;
    targetHeading = newTargetHeading;
    swerveDrive.setTargetHeading(targetHeading.plus(Rotation2d.fromRadians(headingVelocity * SHOOTER_PIVOT.ROTATION_DELAY)));
  }

  public double calculateShotChance(Translation3d speakerPosition) {
    Translation3d shooterLocation = getShooterLocationOnField(swerveDrive.getPose());
    double floorDistance = speakerPosition.toTranslation2d().getDistance(shooterLocation.toTranslation2d());
    double totalDistance = speakerPosition.getDistance(shooterLocation);
    
    double speakerVerticalDegreesOfView = Units.radiansToDegrees(2.0 * Math.atan((Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0 / totalDistance));
    double speakerLateralDegreesOfView = Units.radiansToDegrees(2.0 * Math.atan((Field.SPEAKER_WIDTH - Field.NOTE_LENGTH) / 2.0 / totalDistance));
    speakerVerticalDegreesOfView *= Math.cos(Math.PI / 2.0 - Math.atan((Field.SPEAKER_RED.getZ() - SHOOTER_PIVOT.POSITION.getZ()) / floorDistance) - Units.degreesToRadians(14.0));
    
    double veritcalDegreesOfAccuracy = Units.radiansToDegrees(SHOOTER_PIVOT.ANGLE_PRECISION) * 2.0;
    double lateralDegreesOfAccuracy  = 0.5 * 2.0;
    
    Rotation2d idealHeading = calculateAimingPoint(speakerPosition).toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle();

    double verticalErrorDegrees = Math.abs(targetPivotAngle.minus(getPivotAngle()).getDegrees());
    double lateralErrorDegrees = Math.abs(idealHeading.minus(swerveDrive.getHeading()).getDegrees());

    double verticalValidShotDegrees = Math.min((speakerVerticalDegreesOfView / 2.0 + veritcalDegreesOfAccuracy / 2.0) - verticalErrorDegrees, Math.min(speakerVerticalDegreesOfView, veritcalDegreesOfAccuracy));
    double lateralValidShotDegrees = Math.min((speakerLateralDegreesOfView / 2.0 + lateralDegreesOfAccuracy / 2.0) - lateralErrorDegrees, Math.min(speakerLateralDegreesOfView, lateralDegreesOfAccuracy));
    verticalValidShotDegrees = Math.max(0, verticalValidShotDegrees);
    lateralValidShotDegrees = Math.max(0, lateralValidShotDegrees);

    return (verticalValidShotDegrees * lateralValidShotDegrees) / (veritcalDegreesOfAccuracy * lateralDegreesOfAccuracy);
  }

  public Command calibrate() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          motor.set(volts.in(Volts) / RobotController.getBatteryVoltage());
        },
        log -> {
          log.motor("shooter-pivot")
            .voltage(Volts.of(motor.get() * RobotController.getBatteryVoltage()))
            .linearPosition(Meters.of(encoder.getPosition()))
            .linearVelocity(MetersPerSecond.of(encoder.getVelocity()));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> isCalibrating = true),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> isCalibrating = false)
    );
  }
}
