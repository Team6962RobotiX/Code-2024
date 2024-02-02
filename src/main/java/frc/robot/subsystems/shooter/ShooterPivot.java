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
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
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
  private double targetAngle = 0.0;
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private boolean isCalibrating = false;
  private ShooterWheels shooterWheels;
  private SwerveDrive swerveDrive;
  private double headingVelocity = 0.0;
  private Rotation2d targetHeading = new Rotation2d();

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
  }

  public void setTargetAngle(double angle) {
    targetAngle = angle;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    if (isCalibrating) return;
    
    Translation3d speakerPosition = Field.SPEAKER_RED;
    Translation2d swerveDrivePosition = swerveDrive.getPose().getTranslation();
    Translation3d shooterPositionRotated = SHOOTER_WHEELS.POSITION.rotateBy(new Rotation3d(0.0, 0.0, swerveDrive.getHeading().getRadians()));
    Translation3d shooterPositionOnField = new Translation3d(shooterPositionRotated.getX() + swerveDrivePosition.getX(), shooterPositionRotated.getY() + swerveDrivePosition.getY(), shooterPositionRotated.getZ());
    double distance = speakerPosition.getDistance(shooterPositionOnField);

    Translation2d currentVelocity = swerveDrive.getFieldVelocity();
    Translation2d projectileOffset = currentVelocity.times(distance / shooterWheels.getProjectileVelocity());
    
    Translation3d pointToAimTo = new Translation3d(
      speakerPosition.getX() - projectileOffset.getX(),
      speakerPosition.getY() - projectileOffset.getY(),
      speakerPosition.getZ()
    );
    
    Rotation2d newTargetHeading = pointToAimTo.toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle();
    headingVelocity = newTargetHeading.minus(targetHeading).getRadians() / 0.02;
    targetHeading = newTargetHeading;

    Logger.autoLog("actualTargetHeading", targetHeading.getDegrees());

    if (distance < 5.0) {
      swerveDrive.setTargetHeading(targetHeading.plus(Rotation2d.fromRadians(headingVelocity * SHOOTER_PIVOT.ROTATION_DELAY)));
    }

    // double speakerDistance = 1.0;
    // double speakerHeight = 1.0;
    // setTargetAngle(shooterWheels.calculatePivotAngle());
    // shooterWheels.setTargetAngularVelocity(SHOOTER_WHEELS.TARGET_SPEED);
  }

  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
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
