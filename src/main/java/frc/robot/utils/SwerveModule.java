// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.lang.constant.Constable;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.NEO;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.DRIVE_SMART_MOTION;
import frc.robot.Constants.SWERVE_DRIVE.STEER_SMART_MOTION;
import frc.robot.Constants.SwerveMath;

public class SwerveModule {
  private CANSparkMax driveMotor, steerMotor;
  private RelativeEncoder driveEncoder, steerEncoder;
  private CANCoder absoluteSteerEncoder;
  private SparkMaxPIDController driveController, steerController;
  private SwerveModuleState state = new SwerveModuleState();
  private String name;

  public SwerveModule(int id) {
    name = SWERVE_DRIVE.MODULE_NAMES[id];
    
    // MOTOR SETUP
    driveMotor = new CANSparkMax(CAN.SWERVE_DRIVE_SPARK_MAX[id], MotorType.kBrushless);
    steerMotor = new CANSparkMax(CAN.SWERVE_STEER_SPARK_MAX[id], MotorType.kBrushless);

    Logger.REV(driveMotor.restoreFactoryDefaults(), "driveMotor.restoreFactoryDefaults");
    Logger.REV(steerMotor.restoreFactoryDefaults(), "steerMotor.restoreFactoryDefaults");

    Logger.REV(driveMotor.setIdleMode(IdleMode.kBrake), "driveMotor.setIdleMode");
    Logger.REV(steerMotor.setIdleMode(IdleMode.kBrake), "steerMotor.setIdleMode");

    steerMotor.setInverted(true);

    Logger.REV(driveMotor.setSmartCurrentLimit(SWERVE_DRIVE.MOTOR_CURRENT_LIMIT, NEO.SAFE_STALL_CURRENT), "driveMotor.setSmartCurrentLimit");
    Logger.REV(steerMotor.setSmartCurrentLimit(SWERVE_DRIVE.MOTOR_CURRENT_LIMIT, NEO.SAFE_STALL_CURRENT), "driveMotor.setSmartCurrentLimit");
    Logger.REV(driveMotor.setOpenLoopRampRate(SWERVE_DRIVE.MOTOR_RAMP_RATE_SECONDS), "driveMotor.setOpenLoopRampRate");
    Logger.REV(steerMotor.setOpenLoopRampRate(SWERVE_DRIVE.MOTOR_RAMP_RATE_SECONDS), "steerMotor.setOpenLoopRampRate");
    Logger.REV(driveMotor.setClosedLoopRampRate(SWERVE_DRIVE.MOTOR_RAMP_RATE_SECONDS), "driveMotor.setClosedLoopRampRate");
    Logger.REV(steerMotor.setClosedLoopRampRate(SWERVE_DRIVE.MOTOR_RAMP_RATE_SECONDS), "steerMotor.setClosedLoopRampRate");

    
    // ENCODER SETUP
    absoluteSteerEncoder = new CANCoder(CAN.SWERVE_STEER_CANCODERS[id]);
    steerEncoder = steerMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();

    Logger.REV(driveEncoder.setPositionConversionFactor(SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION), "driveEncoder.setPositionConversionFactor");
    Logger.REV(driveEncoder.setVelocityConversionFactor(SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION / 60.0), "driveEncoder.setVelocityConversionFactor");
    Logger.REV(steerEncoder.setPositionConversionFactor(SWERVE_DRIVE.STEER_MOTOR_RADIANS_PER_REVOLUTION), "steerEncoder.setPositionConversionFactor");
    Logger.REV(steerEncoder.setVelocityConversionFactor(SWERVE_DRIVE.STEER_MOTOR_RADIANS_PER_REVOLUTION / 60.0), "steerEncoder.setVelocityConversionFactor");

    CANCoderConfiguration CANCoderConfig = new CANCoderConfiguration();
    CANCoderConfig.magnetOffsetDegrees = SWERVE_DRIVE.STEER_ENCODER_OFFSETS[id];
    CANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    absoluteSteerEncoder.configAllSettings(CANCoderConfig);

    
    // PID SETUP
    driveController = driveMotor.getPIDController();
    Logger.REV(driveController.setP(DRIVE_SMART_MOTION.kP, 0), "driveController.setP");
    Logger.REV(driveController.setI(DRIVE_SMART_MOTION.kI, 0), "driveController.setI");
    Logger.REV(driveController.setD(DRIVE_SMART_MOTION.kD, 0), "driveController.setD");
    Logger.REV(driveController.setFF(DRIVE_SMART_MOTION.kFF, 0), "driveController.setFF");
    Logger.REV(driveController.setOutputRange(-SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, 0), "driveController.setOutputRange");
    Logger.REV(driveController.setFeedbackDevice(driveEncoder), "driveController.setFeedbackDevice");

    steerController = steerMotor.getPIDController();
    Logger.REV(steerController.setP(STEER_SMART_MOTION.kP, 0), "steerController.setP");
    Logger.REV(steerController.setI(STEER_SMART_MOTION.kI, 0), "steerController.setI");
    Logger.REV(steerController.setD(STEER_SMART_MOTION.kD, 0), "steerController.setD");
    Logger.REV(steerController.setFF(STEER_SMART_MOTION.kFF, 0), "steerController.setFF");
    Logger.REV(steerController.setOutputRange(-SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, 0), "steerController.setOutputRange");
    Logger.REV(steerController.setFeedbackDevice(steerEncoder), "steerController.setFeedbackDevice");
    Logger.REV(steerController.setPositionPIDWrappingEnabled(true), "steerController.setPositionPIDWrappingEnabled");
    Logger.REV(steerController.setPositionPIDWrappingMaxInput(Math.PI), "steerController.setPositionPIDWrappingMaxInput");
    Logger.REV(steerController.setPositionPIDWrappingMinInput(-Math.PI), "steerController.setPositionPIDWrappingMaxInput");

    driveMotor.burnFlash();
    steerMotor.burnFlash(); 
    
    seedSteerEncoder();
  }

  // Set target angle and velocity
  public void drive(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getSteerRotation2d());
    this.state = state;
    
    // Use onboard PIDF controllers
    driveController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    steerController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  /**
   * Seeds the position of the built-in relative encoder with the absolute position of the steer CANCoder.
   * This is because the CANCoder polls at a lower rate than we'd like, so we essentially turn the relative encoder into an fast-updating absolute encoder.
   * Also the built-in SparkMaxPIDControllers require a compatible encoder to run the faster 1kHz closed loop 
   */
  public void seedSteerEncoder() {
    steerEncoder.setPosition(getAbsoluteSteerRadians());
  }

  /**
   * @return Steering direction in radians (-PI - PI)
   */
  public double getSteerRadians() {
    if (Math.abs(steerEncoder.getVelocity()) < 0.01) seedSteerEncoder();
    if (Math.abs(steerEncoder.getPosition()) > Math.PI) steerEncoder.setPosition(SwerveMath.clampRadians(steerEncoder.getPosition()));
    return steerEncoder.getPosition();
  }

  /**
   * @return Steering direction from absolute encoder (CANCoder) in radians (-PI - PI)
   */
  public double getAbsoluteSteerRadians() {
    return Units.degreesToRadians(absoluteSteerEncoder.getAbsolutePosition());
  }

  /**
   * @return Steering direction in degrees (-180 - 180)
   */
  public Rotation2d getSteerRotation2d() {
    return Rotation2d.fromRadians(getSteerRadians());
  }

  /**
   * @return Drive wheel velocity in m/s
   */
  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  /**
   * @return Name of the module (set in constants file)
   */
  public String getName() {
    return name;
  }

  /**
   * @return The total current of both motors (measured in Amps)
   */
  public double getCurrent() {
    return driveMotor.getOutputCurrent() + steerMotor.getOutputCurrent();
  }

  /**
   * Stops power to both motors (not persistent)
   */
  public void stop() {
    steerMotor.stopMotor();
    driveMotor.stopMotor();
  }
  
  /**
   * @return The target SwerveModuleState
   */
  public SwerveModuleState getTargetState() {
    return state;
  }

  /**
   * @return The measured SwerveModuleState
   */
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(getVelocity(), getSteerRotation2d());
  }

  /**
   * @return The measured SwerveModulePosition
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getSteerRotation2d());
  }

  /**
   * Logs all values, motor controllers, and encoders to network tables to be saved later
   */
  public void log(String path) {
    Logger.logSparkMax(path + "/driveMotor", driveMotor);
    Logger.logSparkMax(path + "/steerMotor", steerMotor);
    Logger.logCANCoder(path + "/canCoder", absoluteSteerEncoder);
    Logger.logValue(path + "/steerDegrees", getSteerRotation2d().getDegrees());
    Logger.logValue(path + "/velocity", getVelocity());
    Logger.logValue(path + "/name", name);
  }

  /**
   * @param power Motor power (-1.0 - 1.0)
   * @return Drive velocity
   */
  public static double powerToDriveVelocity(double power) {
    return power / DRIVE_SMART_MOTION.kFF;
  }

  public SparkMaxPIDController getDrivePIDFController() {
    return driveController;
  }
  
  public SparkMaxPIDController getSteerPIDFController() {
    return steerController;
  }

  public double getTargetVelocity() {
    return state.speedMetersPerSecond;
  }

  public double getTargetAngle() {
    return state.angle.getRadians();
  }
}