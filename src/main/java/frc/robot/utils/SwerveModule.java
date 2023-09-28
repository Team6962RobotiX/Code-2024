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
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveMath;

public class SwerveModule {
  private CANSparkMax driveMotor;
  private RelativeEncoder relativeDriveEncoder;
  private CANSparkMax steerMotor;
  private RelativeEncoder relativeSteerEncoder;
  private CANCoder absoluteSteerEncoder;
  public PIDController steerController = new PIDController(0.0, 0.0, 0.0);
  private SwerveModuleState state = new SwerveModuleState();
  private String name;
  SlewRateLimiter accelerationLimiter = new SlewRateLimiter(SwerveDriveConstants.WHEEL_MAX_ACCELERATION);

  public SwerveModule(int id) {
    name = SwerveDriveConstants.MODULE_NAMES[id];

    driveMotor = new CANSparkMax(CAN.SWERVE_DRIVE[id], MotorType.kBrushless);
    steerMotor = new CANSparkMax(CAN.SWERVE_STEER[id], MotorType.kBrushless);

    absoluteSteerEncoder = new CANCoder(CAN.SWERVE_STEER_CANCODER[id]);
    relativeDriveEncoder = driveMotor.getEncoder();
    relativeSteerEncoder = steerMotor.getEncoder();

    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    driveMotor.setIdleMode(IdleMode.kBrake);
    steerMotor.setIdleMode(IdleMode.kBrake);

    steerMotor.setInverted(true);

    driveMotor.setSmartCurrentLimit((int) (SwerveDriveConstants.TOTAL_CURRENT_LIMIT / 4.0 * (2.0 / 3.0)));
    steerMotor.setSmartCurrentLimit((int) (SwerveDriveConstants.TOTAL_CURRENT_LIMIT / 4.0 * (1.0 / 3.0)));

    driveMotor.setOpenLoopRampRate(SwerveDriveConstants.MOTOR_RAMP_RATE_SECONDS);
    steerMotor.setOpenLoopRampRate(SwerveDriveConstants.MOTOR_RAMP_RATE_SECONDS);

    relativeDriveEncoder.setPositionConversionFactor(SwerveDriveConstants.DRIVE_METERS_PER_MOTOR_ROTATION);
    relativeDriveEncoder.setVelocityConversionFactor(SwerveDriveConstants.DRIVE_METERS_PER_MOTOR_ROTATION / 60.0);
    relativeSteerEncoder.setPositionConversionFactor(SwerveDriveConstants.STEER_RADIANS_PER_MOTOR_ROTATION);
    relativeSteerEncoder.setVelocityConversionFactor(SwerveDriveConstants.STEER_RADIANS_PER_MOTOR_ROTATION / 60.0);

    CANCoderConfiguration CANCoderConfig = new CANCoderConfiguration();
    CANCoderConfig.magnetOffsetDegrees = SwerveDriveConstants.STEER_ENCODER_OFFSETS[id];
    CANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    absoluteSteerEncoder.configAllSettings(CANCoderConfig);

    steerController.setTolerance(Units.degreesToRadians(SwerveDriveConstants.MODULE_STEER_PID_TOLERANCE));
    steerController.enableContinuousInput(-Math.PI, Math.PI);
    steerController.setPID(
        SwerveDriveConstants.MODULE_STEER_PID[0],
        SwerveDriveConstants.MODULE_STEER_PID[1],
        SwerveDriveConstants.MODULE_STEER_PID[2]);
    selfCheck();
  }

  // Set target angle and velocity
  public void drive(SwerveModuleState state) {
    this.state = SwerveModuleState.optimize(state, getSteerDirection());
    
    // Calculate motor power from velocities
    double drivePower = SwerveMath.wheelVelocityToMotorPower(state.speedMetersPerSecond);
    double steerPower = SwerveMath.steerVelocityToMotorPower(steerController.calculate(getSteerDirection().getRadians(), state.angle.getRadians()));
    
    // Clamp power to be within the motor power hard cap
    drivePower = Math.abs(drivePower) > SwerveDriveConstants.MOTOR_POWER_HARD_CAP ? SwerveDriveConstants.MOTOR_POWER_HARD_CAP * Math.signum(drivePower) : drivePower;
    steerPower = Math.abs(steerPower) > SwerveDriveConstants.MOTOR_POWER_HARD_CAP ? SwerveDriveConstants.MOTOR_POWER_HARD_CAP * Math.signum(steerPower) : steerPower;
    
    // Limit wheel acceleration
    drivePower = SwerveMath.wheelVelocityToMotorPower(accelerationLimiter.calculate(SwerveMath.motorPowerToWheelVelocity(drivePower)));
    
    // Drive motors
    driveMotor.set(drivePower);
    steerMotor.set(steerPower);
  }

  // Get the direction of the steering wheel (-180 - 180)
  public Rotation2d getSteerDirection() {
    return Rotation2d.fromDegrees(absoluteSteerEncoder.getAbsolutePosition());
  }

  // Get velocity in m/s
  public double getVelocity() {
    return relativeDriveEncoder.getVelocity();
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
    return new SwerveModulePosition(relativeDriveEncoder.getPosition(), getSteerDirection());
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