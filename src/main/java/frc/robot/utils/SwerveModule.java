// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.*;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

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

    driveMotor.setOpenLoopRampRate(SwerveDriveConstants.TIME_TO_FULL_POWER);
    steerMotor.setOpenLoopRampRate(SwerveDriveConstants.TIME_TO_FULL_POWER);

    relativeDriveEncoder.setPositionConversionFactor(SwerveDriveConstants.DRIVE_METERS_PER_MOTOR_ROTATION);
    relativeDriveEncoder.setVelocityConversionFactor(SwerveDriveConstants.DRIVE_METERS_PER_MOTOR_ROTATION / 60);
    relativeSteerEncoder.setPositionConversionFactor(SwerveDriveConstants.STEER_RADIANS_PER_MOTOR_ROTATION);
    relativeSteerEncoder.setVelocityConversionFactor(SwerveDriveConstants.STEER_RADIANS_PER_MOTOR_ROTATION / 60);

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
  public void setState(SwerveModuleState state) {
    this.state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getSteerRadians()));
  }

  // Drive motors to approximate target angle and velocity
  public void drive() { // Must be called periodically
    double drivePower = SwerveMath.wheelVelocityToMotorPower(state.speedMetersPerSecond);
    double steerPower = SwerveMath.steerVelocityToMotorPower(steerController.calculate(getSteerRadians(), state.angle.getRadians()));

    if (Math.abs(drivePower) > SwerveDriveConstants.MOTOR_POWER_HARD_CAP) {
      drivePower = SwerveDriveConstants.MOTOR_POWER_HARD_CAP * Math.signum(drivePower);
    }

    if (Math.abs(steerPower) > SwerveDriveConstants.MOTOR_POWER_HARD_CAP) {
      steerPower = SwerveDriveConstants.MOTOR_POWER_HARD_CAP * Math.signum(steerPower);
    }

    drivePower = SwerveMath.wheelVelocityToMotorPower(accelerationLimiter.calculate(SwerveMath.motorPowerToWheelVelocity(drivePower)));

    driveMotor.set(drivePower);
    steerMotor.set(steerPower);
  }

  // Get the direction of the steering wheel (-PI - PI)
  public double getSteerRadians() {
    return getSteerDegrees() / 180.0 * Math.PI;
  }

  // Get the direction of the steering wheel (-180 - 180)
  public double getSteerDegrees() {
    return absoluteSteerEncoder.getAbsolutePosition();
  }

  // Get velocity in m/s
  public double getVelocity() {
    return relativeDriveEncoder.getVelocity();
  }

  // Get distance traveled in m
  public double getDistanceTraveled() {
    return relativeDriveEncoder.getPosition();
  }

  // Get current angle and velocity (SwerveModulePosition)
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(relativeDriveEncoder.getPosition(), Rotation2d.fromRadians(getSteerRadians()));
  }

  // Get name of module
  public String getName() {
    return name;
  }

  // Get total voltage through both motors
  public double getVoltage() {
    return driveMotor.getBusVoltage() + steerMotor.getBusVoltage();
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

  public void selfCheck() {
    SelfCheck.checkMotorFaults(new CANSparkMax[] { driveMotor, steerMotor });
    SelfCheck.checkCANCoderFaults(absoluteSteerEncoder);
  }

  public SwerveModuleState getState() {
    return state;
  }

  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(getVelocity(), Rotation2d.fromRadians(getSteerRadians()));
  }

  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }

  public CANSparkMax getSteerMotor() {
    return steerMotor;
  }

  public CANCoder getCanCoder() {
    return absoluteSteerEncoder;
  }
}