// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.NEO;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.DRIVE_MOTOR_CONFIG;
import frc.robot.Constants.SWERVE_DRIVE.STEER_MOTOR_CONFIG;
import frc.robot.Constants.SWERVE_MATH;
import frc.robot.util.ConfigUtils;
import frc.robot.util.Logging.Logger;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax driveMotor, steerMotor;
  private RelativeEncoder driveEncoder, steerEncoder;
  private CANCoder absoluteSteerEncoder;
  private SparkMaxPIDController drivePID, steerPID;
  private SwerveModuleState state = new SwerveModuleState();
  public int id;

  public SwerveModule(int id) {
    this.id = id;

    driveMotor           = new CANSparkMax(CAN.SWERVE_DRIVE_SPARK_MAX[id], MotorType.kBrushless);
    steerMotor           = new CANSparkMax(CAN.SWERVE_STEER_SPARK_MAX[id], MotorType.kBrushless);
    absoluteSteerEncoder = new CANCoder(CAN.SWERVE_STEER_CANCODERS[id]);
    steerEncoder         = steerMotor.getEncoder();
    driveEncoder         = driveMotor.getEncoder();
    drivePID             = driveMotor.getPIDController();
    steerPID             = steerMotor.getPIDController();

    ConfigUtils.configure(driveMotor, List.of(
      () -> driveMotor.restoreFactoryDefaults(),
      () -> driveMotor.setIdleMode(IdleMode.kBrake),
      () -> driveMotor.enableVoltageCompensation(12.0),
      () -> driveMotor.setSmartCurrentLimit(Math.min(DRIVE_MOTOR_CONFIG.CURRENT_LIMIT, NEO.SAFE_STALL_CURRENT), DRIVE_MOTOR_CONFIG.CURRENT_LIMIT),
      () -> driveMotor.setClosedLoopRampRate(DRIVE_MOTOR_CONFIG.RAMP_RATE),
      () -> driveEncoder.setPositionConversionFactor(SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION),
      () -> driveEncoder.setVelocityConversionFactor(SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION / 60.0),
      () -> drivePID.setP(DRIVE_MOTOR_CONFIG.kP, 0),
      () -> drivePID.setI(DRIVE_MOTOR_CONFIG.kI, 0),
      () -> drivePID.setD(DRIVE_MOTOR_CONFIG.kD, 0),
      () -> drivePID.setFF(DRIVE_MOTOR_CONFIG.kFF, 0),
      () -> drivePID.setOutputRange(-SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, 0),
      () -> driveMotor.burnFlash(),
      () -> ConfigUtils.setPeriodicFramePeriods(driveMotor, DRIVE_MOTOR_CONFIG.statusFramePeriods)
    ));

    ConfigUtils.configure(steerMotor, List.of(
      () -> steerMotor.restoreFactoryDefaults(),
      () -> steerMotor.setIdleMode(IdleMode.kBrake),
      () -> steerMotor.enableVoltageCompensation(12.0),
      () -> steerMotor.setSmartCurrentLimit(Math.min(STEER_MOTOR_CONFIG.CURRENT_LIMIT, NEO.SAFE_STALL_CURRENT), STEER_MOTOR_CONFIG.CURRENT_LIMIT),
      () -> steerMotor.setClosedLoopRampRate(STEER_MOTOR_CONFIG.RAMP_RATE),
      () -> steerEncoder.setPositionConversionFactor(SWERVE_DRIVE.STEER_MOTOR_RADIANS_PER_REVOLUTION),
      () -> steerEncoder.setVelocityConversionFactor(SWERVE_DRIVE.STEER_MOTOR_RADIANS_PER_REVOLUTION / 60.0),
      () -> steerPID.setP(STEER_MOTOR_CONFIG.kP, 0),
      () -> steerPID.setI(STEER_MOTOR_CONFIG.kI, 0),
      () -> steerPID.setD(STEER_MOTOR_CONFIG.kD, 0),
      () -> steerPID.setPositionPIDWrappingEnabled(true),
      () -> steerPID.setPositionPIDWrappingMinInput(-Math.PI),
      () -> steerPID.setPositionPIDWrappingMinInput(Math.PI),
      () -> steerPID.setOutputRange(-SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, 0),
      () -> steerMotor.burnFlash(),
      () -> ConfigUtils.setPeriodicFramePeriods(steerMotor, STEER_MOTOR_CONFIG.statusFramePeriods)
    ));

    ConfigUtils.configure(absoluteSteerEncoder, List.of(
      () -> absoluteSteerEncoder.configMagnetOffset(SWERVE_DRIVE.STEER_ENCODER_OFFSETS[id]),
      () -> absoluteSteerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
    ));
    
    // driveMotor.setClosedLoopRampRate(Math.max((NEO.FREE_SPEED / 60.0) / ((9.80 * SWERVE_DRIVE.COEFFICIENT_OF_FRICTION) / SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION), DRIVE_MOTOR_CONFIG.RAMP_RATE))

    seedSteerEncoder();

    Logger.autoLog("current",                 () -> this.getTotalCurrent());
    Logger.autoLog("measuredAngle",           () -> this.getMeasuredState().angle);
    Logger.autoLog("measuredVelocity",        () -> this.getMeasuredState().speedMetersPerSecond);
    Logger.autoLog("targetAngle",             () -> this.getTargetState().angle);
    Logger.autoLog("targetVelocity",          () -> this.getTargetState().speedMetersPerSecond);
    Logger.autoLog("getAbsoluteSteerDegrees", () -> this.getTrueSteerDirection().getDegrees());
    Logger.autoLog("measuredState",           () -> this.getMeasuredState());
    Logger.autoLog("targetState",             () -> this.getTargetState());
  }

  public void drive(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getMeasuredState().angle);

    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) {
      state.speedMetersPerSecond *= Math.cos(SWERVE_MATH.angleDistance(state.angle.getRadians(), getMeasuredState().angle.getRadians()));
    }

    this.state = state;
    
    if (Math.abs(getMeasuredState().speedMetersPerSecond) < SWERVE_DRIVE.VELOCITY_DEADBAND && SWERVE_MATH.angleDistance(this.state.angle.getRadians(), steerEncoder.getPosition()) < Units.degreesToRadians(1.0)) {
      seedSteerEncoder();
    }

    // Use onboard motion profiling controllers
    drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    steerPID.setReference(state.angle.getRadians(),   ControlType.kPosition);
  }
  
  public void stop() {
    steerMotor.stopMotor();
    driveMotor.stopMotor();
  }

  /**
   * Seeds the position of the built-in relative encoder with the absolute position of the steer CANCoder.
   * This is because the CANCoder polls at a lower rate than we'd like, so we essentially turn the relative encoder into an fast-updating absolute encoder.
   * Also the built-in SparkMaxPIDControllers require a compatible encoder to run the faster 1kHz closed loop 
   */
  private void seedSteerEncoder() {
    steerEncoder.setPosition(getTrueSteerDirection().getRadians());
  }

  private Rotation2d getTrueSteerDirection() {
    return Rotation2d.fromDegrees(absoluteSteerEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getTargetState() {
    return state;
  }

  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromRadians(MathUtil.angleModulus(steerEncoder.getPosition())));
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getMeasuredState().angle);
  }

  public static double calcDriveVelocity(double power) {
    return power * (NEO.FREE_SPEED / 60.0 * SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION);
  }

  public double getTotalCurrent() {
    return driveMotor.getOutputCurrent() + steerMotor.getOutputCurrent();
  }

  public void runCharacterization(double volts) {
    steerPID.setReference(0.0, ControlType.kPosition);
    driveMotor.setVoltage(volts);
  }

  public Pose2d getPose(Pose2d robotPose) {
    Pose2d relativePose = new Pose2d();
    if (id == 0) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (id == 1) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (id == 2) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (id == 3) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    return relativePose.relativeTo(new Pose2d(
      new Translation2d(),
      robotPose.getRotation().times(-1.0)
    )).relativeTo( new Pose2d(
      -robotPose.getX(),
      -robotPose.getY(),
      new Rotation2d()
    ));
  }
}