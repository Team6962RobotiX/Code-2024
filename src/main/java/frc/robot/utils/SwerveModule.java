// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveMath;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.SelfCheck.*;

public class SwerveModule {
  private CANSparkMax driveMotor;
  private CANSparkMax steerMotor;
  private RelativeEncoder driveEncoder;
  private RelativeEncoder steerEncoder;
  private CANCoder absoluteSteerEncoder;
  public PIDController steerPID = new PIDController(SwerveDriveConstants.MODULE_STEER_PID[0], SwerveDriveConstants.MODULE_STEER_PID[1], SwerveDriveConstants.MODULE_STEER_PID[2]);
  private SwerveModuleState state;
  private String name;
  SlewRateLimiter accelerationLimiter = new SlewRateLimiter(SwerveDriveConstants.WHEEL_MAX_ACCELERATION);

  public SwerveModule(int id) {
    name = SwerveDriveConstants.MODULE_NAMES[id];

    // MOTOR SETUP
    driveMotor = new CANSparkMax(CAN.SWERVE_DRIVE[id], MotorType.kBrushless);
    steerMotor = new CANSparkMax(CAN.SWERVE_STEER[id], MotorType.kBrushless);

    Logger.REV(driveMotor.restoreFactoryDefaults(), "driveMotor.restoreFactoryDefaults");
    Logger.REV(steerMotor.restoreFactoryDefaults(), "steerMotor.restoreFactoryDefaults");

    Logger.REV(driveMotor.setIdleMode(IdleMode.kBrake), "driveMotor.setIdleMode");
    Logger.REV(steerMotor.setIdleMode(IdleMode.kBrake), "steerMotor.setIdleMode");

    steerMotor.setInverted(true);

    Logger.REV(driveMotor.setSmartCurrentLimit((int) (SwerveDriveConstants.TOTAL_CURRENT_LIMIT / 4.0)), "driveMotor.setSmartCurrentLimit");
    Logger.REV(driveMotor.setOpenLoopRampRate(SwerveDriveConstants.MOTOR_RAMP_RATE_SECONDS), "driveMotor.setOpenLoopRampRate");
    Logger.REV(steerMotor.setOpenLoopRampRate(SwerveDriveConstants.MOTOR_RAMP_RATE_SECONDS), "steerMotor.setOpenLoopRampRate");

    // ENCODER SETUP
    absoluteSteerEncoder = new CANCoder(CAN.SWERVE_STEER_CANCODER[id]);
    steerEncoder = steerMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();

    Logger.REV(driveEncoder.setPositionConversionFactor(SwerveDriveConstants.DRIVE_METERS_PER_MOTOR_ROTATION), "driveEncoder.setPositionConversionFactor");
    Logger.REV(driveEncoder.setVelocityConversionFactor(SwerveDriveConstants.DRIVE_METERS_PER_MOTOR_ROTATION / 60.0), "driveEncoder.setVelocityConversionFactor");
    Logger.REV(steerEncoder.setPositionConversionFactor(SwerveDriveConstants.STEER_RADIANS_PER_MOTOR_ROTATION), "steerEncoder.setPositionConversionFactor");
    Logger.REV(steerEncoder.setVelocityConversionFactor(SwerveDriveConstants.STEER_RADIANS_PER_MOTOR_ROTATION / 60.0), "steerEncoder.setVelocityConversionFactor");

    CANCoderConfiguration CANCoderConfig = new CANCoderConfiguration();
    CANCoderConfig.magnetOffsetDegrees = SwerveDriveConstants.STEER_ENCODER_OFFSETS[id];
    CANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    absoluteSteerEncoder.configAllSettings(CANCoderConfig);

    // PID SETUP
    steerPID.enableContinuousInput(-Math.PI, Math.PI);
    steerPID.setTolerance(Units.degreesToRadians(SwerveDriveConstants.MODULE_STEER_PID_TOLERANCE));
    
    driveMotor.burnFlash();
    steerMotor.burnFlash();
    
    seedSteerEncoder();

    selfCheck();
  }

  // Set target angle and velocity
  public void drive(SwerveModuleState state) {
    this.state = SwerveModuleState.optimize(state, getSteerDirection());

    // Calculate motor power from velocities
    double drivePower = SwerveMath.wheelVelocityToMotorPower(state.speedMetersPerSecond);
    double steerPower = SwerveMath.steerVelocityToMotorPower(steerPID.calculate(getSteerDirection().getRadians(), state.angle.getRadians()));
    
    // Clamp power to be within the motor power hard cap
    drivePower = Math.abs(drivePower) > SwerveDriveConstants.MOTOR_POWER_HARD_CAP ? SwerveDriveConstants.MOTOR_POWER_HARD_CAP * Math.signum(drivePower) : drivePower;
    steerPower = Math.abs(steerPower) > SwerveDriveConstants.MOTOR_POWER_HARD_CAP ? SwerveDriveConstants.MOTOR_POWER_HARD_CAP * Math.signum(steerPower) : steerPower;

    // Limit wheel acceleration
    drivePower = SwerveMath.wheelVelocityToMotorPower(accelerationLimiter.calculate(SwerveMath.motorPowerToWheelVelocity(drivePower)));

    // Drive motors
    driveMotor.setVoltage(drivePower * SwerveDriveConstants.MAX_VOLTAGE);
    steerMotor.setVoltage(steerPower * SwerveDriveConstants.MAX_VOLTAGE);
  }

  public void seedSteerEncoder() {
    steerEncoder.setPosition(Units.degreesToRadians(absoluteSteerEncoder.getAbsolutePosition()));
  }

  // Get the direction of the steering wheel (-180 - 180)
  public Rotation2d getSteerDirection() {
    return Rotation2d.fromRadians(absoluteSteerEncoder.getAbsolutePosition());
  }

  // Get velocity in m/s
  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  // Get name of module
  public String getName() {
    return name;
  }

  // Get total current through both motors
  public double getCurrent() {
    return driveMotor.getOutputCurrent() + steerMotor.getOutputCurrent();
  }

  // Stop power to both motors
  public void stop() {
    steerMotor.set(0.0);
    driveMotor.set(0.0);
  }

  public SwerveModuleState getTargetState() {
    return state;
  }

  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(getVelocity(), getSteerDirection());
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getSteerDirection());
  }

  public CANSparkMax[] getMotors() {
    return new CANSparkMax[] { driveMotor, steerMotor };
  }

  public CANCoder getCanCoder() {
    return absoluteSteerEncoder;
  }

  public void selfCheck() {
    SelfCheck.checkMotorFaults(getMotors());
    SelfCheck.checkCANCoderFaults(getCanCoder());
  }
}