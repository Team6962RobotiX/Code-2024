// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
  private SwerveModuleState targetState;
  private double targetDriveVelocity = 0.0;
  private String name;
  private boolean isSteerCalibrated = false;

  SwerveModule(int id) {
    driveMotor = new CANSparkMax(CAN.SWERVE_DRIVE[id], MotorType.kBrushless);
    steerMotor = new CANSparkMax(CAN.SWERVE_STEER[id], MotorType.kBrushless);
    absoluteSteerEncoder = new CANCoder(CAN.SWERVE_STEER_CANCODER[id], "rio");
    name = SwerveDriveConfig.MODULE_NAMES[id];

    relativeDriveEncoder = driveMotor.getEncoder();
    relativeSteerEncoder = steerMotor.getEncoder();

    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    driveMotor.setSmartCurrentLimit((int) (SwerveDriveConfig.TOTAL_CURRENT_LIMIT / 4.0 * (2.0 / 3.0)));
    steerMotor.setSmartCurrentLimit((int) (SwerveDriveConfig.TOTAL_CURRENT_LIMIT / 4.0 * (1.0 / 3.0)));
    driveMotor.setOpenLoopRampRate(SwerveDriveConfig.MOTOR_POWER_RAMP_RATE);
    steerMotor.setOpenLoopRampRate(SwerveDriveConfig.MOTOR_POWER_RAMP_RATE);

    CANCoderConfiguration CANCoderConfig = new CANCoderConfiguration();
    CANCoderConfig.sensorCoefficient = 360.0 / 4096.0;
    CANCoderConfig.unitString = "degrees";
    CANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    CANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    CANCoderConfig.magnetOffsetDegrees = SwerveDriveConfig.STEER_ENCODER_OFFSETS[id];
    CANCoderConfig.sensorDirection = false;

    absoluteSteerEncoder.configAllSettings(CANCoderConfig);
    absoluteSteerEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
    absoluteSteerEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10);

    relativeDriveEncoder.setVelocityConversionFactor(SwerveDriveConfig.MOTOR_RPM_VELOCITY_RATIO);
    relativeSteerEncoder.setPositionConversionFactor(SwerveDriveConfig.STEER_GEAR_REDUCTION * 360);

    steerController.enableContinuousInput(0.0, 360.0);
    setPID(SwerveDriveConfig.MODULE_STEER_PID);

    SelfCheck.checkMotorFaults(new CANSparkMax[] { driveMotor, steerMotor });
    calibrateSteerAngle();
  }

  // Set target angle and velocity
  public void setTargetState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));

    targetState = state;

    double targetVelocity = state.speedMetersPerSecond;
    if (Math.abs(targetVelocity) < SwerveDriveConfig.VELOCITY_DEADZONE) {
      targetVelocity = 0.0;
    }
    double targetAngle = state.angle.getDegrees();

    // System.out.println("target angle " + name + ": " + targetAngle);
    System.out.println("current angle " + name + ": " + getAngle());

    setTargetVelocity(targetVelocity);
    setTargetAngle(targetAngle);
  }

  // Drive motors to approximate target angle and velocity
  public void drive() { // Must be called periodically
    double drivePower = driveVelocityToMotorPower(targetDriveVelocity);
    double steerPower = steerController.calculate(getAngle());

    driveMotor.set(Math.min(drivePower, SwerveDriveConfig.MOTOR_POWER_LIMIT));
    steerMotor.set(Math.min(steerPower, SwerveDriveConfig.MOTOR_POWER_LIMIT));

    if (driveMotor.get() == 0 && steerMotor.get() == 0) {
      if (!isSteerCalibrated) {
        calibrateSteerAngle();
      }
    } else {
      isSteerCalibrated = false;
    }

    targetDriveVelocity = 0.0;
  }

  public void setTargetAngle(double angle) {
    // steerController.reset();
    steerController.setSetpoint(angle);
  }

  public void setTargetVelocity(double velocity) {
    targetDriveVelocity = velocity;
  }

  // For PID Tuning, only use in testing mode
  public void setPID(double[] PID) {
    steerController.setPID(PID[0], PID[1], PID[2]);
  }

  // Get the direction of the steering wheel (0 - 180)
  public double getAngle() {
    double angle = relativeSteerEncoder.getPosition();
    relativeSteerEncoder.setPosition(angle % 360);
    return angle;
  }

  public void calibrateSteerAngle() {
    relativeSteerEncoder.setPosition(absoluteSteerEncoder.getAbsolutePosition());
    isSteerCalibrated = true;
  }

  // Get the direction of the steering wheel (0 - 360)
  public double getDriveDirection() {
    if (driveMotor.get() < 0) {
      return (getAngle() + 180.0) % 360.0;
    }
    return getAngle();
  }

  // Get velocity in m/s
  public double getDriveVelocity() {
    return relativeDriveEncoder.getVelocity();
  }

  // Convert velocity in m/s to motor power from 0 - 1
  public static double driveVelocityToMotorPower(double velocity) {
    return velocity / SwerveDriveConfig.FULL_POWER_VELOCITY;
  }

  // Get current angle and velocity (SwerveModuleState) 
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  // Get current angle and velocity (SwerveModulePosition)
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveVelocity(),
        Rotation2d.fromDegrees(getAngle()));
  }

  // Get target angle and velocity
  public SwerveModuleState getTargetState() {
    return targetState;
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

  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }

  public CANSparkMax getSteerMotor() {
    return steerMotor;
  }

  public void selfCheck() {
    SelfCheck.checkMotorFaults(new CANSparkMax[] { driveMotor, steerMotor });
  }
}