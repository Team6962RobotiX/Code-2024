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
import frc.robot.Constants.SwerveDriveConfig;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class SwerveModule {
  private CANSparkMax driveMotor;
  private RelativeEncoder driveEncoder;
  private CANSparkMax steerMotor;
  private CANCoder steerEncoder;
  public PIDController steerController = new PIDController(0.0, 0.0, 0.0);
  private SwerveModuleState targetState;
  private double targetDriveVelocity = 0.0;
  private String name;

  SwerveModule(CANSparkMax driveMotor, CANSparkMax steerMotor, CANCoder steerEncoder, String name) {
    this.driveMotor = driveMotor;
    this.driveEncoder = driveMotor.getEncoder();
    this.steerMotor = steerMotor;
    this.steerEncoder = steerEncoder;
    this.name = name;

    this.driveMotor.restoreFactoryDefaults();
    this.steerMotor.restoreFactoryDefaults();

    this.driveMotor.setSmartCurrentLimit(SwerveDriveConfig.TOTAL_CURRENT_LIMIT / 4 * (2 / 3));
    this.steerMotor.setSmartCurrentLimit(SwerveDriveConfig.TOTAL_CURRENT_LIMIT / 4 * (1 / 3));
    this.driveMotor.setOpenLoopRampRate(SwerveDriveConfig.MOTOR_POWER_RAMP_RATE);
    this.steerMotor.setOpenLoopRampRate(SwerveDriveConfig.MOTOR_POWER_RAMP_RATE);

    CANCoderConfiguration CANCoderConfig = new CANCoderConfiguration();
    CANCoderConfig.sensorCoefficient = 360 / 4096.0;
    CANCoderConfig.unitString = "degrees";
    CANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    steerEncoder.configAllSettings(CANCoderConfig);

    this.driveEncoder.setVelocityConversionFactor(SwerveDriveConfig.MOTOR_RPM_VELOCITY_RATIO);

    setPID(SwerveDriveConfig.MODULE_STEER_PID);

    this.steerController.enableContinuousInput(0.0, 360.0);
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
    return steerEncoder.getAbsolutePosition();
  }

  // Get the direction of the steering wheel (0 - 360)
  public double getDriveDirection() {
    if (driveMotor.get() < 0) {
      return (getAngle() + 180) % 360;
    }
    return getAngle();
  }

  // Get velocity in m/s
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
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
}