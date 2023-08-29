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

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
  private CANSparkMax driveMotor;
  private RelativeEncoder driveEncoder;
  private CANSparkMax steerMotor;
  private CANCoder steerEncoder;
  public PIDController steerController = new PIDController(0.0, 0.0, 0.0);
  private SwerveModuleState targetState;
  private double targetDriveSpeed = 0.0;
  private String name;

  SwerveModule(CANSparkMax driveMotor, CANSparkMax steerMotor, CANCoder steerEncoder, String name) {
    this.driveMotor = driveMotor;
    this.driveEncoder = driveMotor.getEncoder();
    this.steerMotor = steerMotor;
    this.steerEncoder = steerEncoder;
    this.name = name;

    driveMotor.setSmartCurrentLimit(Constants.SWERVE_TOTAL_AMP_LIMIT / 8);
    driveMotor.setOpenLoopRampRate(Constants.SWERVE_POWER_RAMP_RATE);

    steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    setPID(Constants.SWERVE_STEER_PID);

    steerController.enableContinuousInput(0.0, 360.0);
  }

  public void setTargetState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));

    targetState = state;

    double targetDriveSpeed = state.speedMetersPerSecond;
    double targetSteerAngle = state.angle.getDegrees();

    setTargetSpeed(targetDriveSpeed);
    setTargetAngle(targetSteerAngle);
  }

  public void drive() {
    double drivePower = driveSpeedToMotorPower(targetDriveSpeed);
    double steerPower = steerController.calculate(getAngle());

    driveMotor.set(drivePower);
    steerMotor.set(steerPower);

    targetDriveSpeed = 0.0;
  }

  public void setTargetAngle(double angle) {
    // steerController.reset();
    steerController.setSetpoint(angle);
  }

  public void setTargetSpeed(double speed) {
    targetDriveSpeed = speed;
  }

  public void setPID(double[] PID) {
    steerController.setPID(PID[0], PID[1], PID[2]);
  }

  public double getAngle() {
    return steerEncoder.getAbsolutePosition();
  }

  public double getDriveDirection() {
    if (targetDriveSpeed < 0) {
      return (steerEncoder.getAbsolutePosition() + 180) % 360;
    }
    return steerEncoder.getAbsolutePosition();
  }

  public double getDriveSpeed() {
    return motorRPMtoDriveSpeed(getMotorRPM());
  }

  public double getMotorRPM() {
    return driveEncoder.getVelocity();
  }

  public static double motorRPMtoDriveSpeed(double RPM) {
    return (RPM * Constants.SWERVE_GEAR_REDUCTION) / 60.0 * Constants.SWERVE_WHEEL_DIAMETER * Math.PI;
  }

  public static double driveSpeedToMotorPower(double speed) {
    return speed / motorRPMtoDriveSpeed(Constants.SWERVE_FULL_POWER_NEO_RPM);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), Rotation2d.fromDegrees(getAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveSpeed(),
        Rotation2d.fromDegrees(getAngle()));
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }

  public String getName() {
    return name;
  }

  public double getVoltage() {
    return driveMotor.getBusVoltage() + steerMotor.getBusVoltage();
  }

  public double getCurrent() {
    return driveMotor.getOutputCurrent() + steerMotor.getOutputCurrent();
  }
}