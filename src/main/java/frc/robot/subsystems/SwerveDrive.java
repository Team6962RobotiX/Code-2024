// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.util.datalog.*;

import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveMath;
import frc.robot.subsystems.*;
import frc.robot.utils.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule[] swerveModules = new SwerveModule[4];
  private AHRS gyro;
  private SwerveDriveKinematics kinematics = SwerveMath.getKinematics();
  private SwerveDriveOdometry odometer;
  private SlewRateLimiter accelerationLimiter = new SlewRateLimiter(SwerveDriveConstants.DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter angularAccelerationLimiter = new SlewRateLimiter(SwerveDriveConstants.DRIVE_MAX_ANGULAR_ACCELERATION);
  private double driveDirection = 0.0;

  public SwerveDrive() {
    try {
      gyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      SelfCheck.warn("Error instantiating navX-MXP:  " + ex.getMessage());
    }

    for (int i = 0; i < 4; i++) {
      swerveModules[i] = new SwerveModule(i);
    }

    odometer = new SwerveDriveOdometry(
        kinematics,
        getRotation2d(),
        getModulePositions(),
        SwerveDriveConstants.STARTING_POSE);

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveModules();

    boolean stopped = true;
    for (SwerveModule module : swerveModules) {
      if (module.getVelocity() > SwerveDriveConstants.VELOCITY_DEADZONE) {
        stopped = false;
        break;
      }
    }

    if (stopped) {
      groundModules();
    }

    // groundModules();
    odometer.update(getRotation2d(), getModulePositions());

    selfCheckModules();
    // SelfCheck.checkPDPFaults();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Get gyro Rotation2d heading
  public AHRS getGyro() {
    return gyro;
  }

  // Get gyro Rotation2d heading
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  public SwerveDriveOdometry getOdometer() {
    return odometer;
  }

  // Get gyro degree heading (-180 - 180)
  public double getHeading() {
    return getRotation2d().getDegrees() % 180.0;
  }

  // Reset gyro heading
  public void zeroHeading() {
    gyro.reset();
    gyro.setAngleAdjustment(SwerveDriveConstants.STARTING_ANGLE_OFFSET);
  }

  // Drive the robot relative to itself
  public void robotOrientedDrive(double forwardVelocity, double strafeVelocity, double angularVelocity) { // m/s and rad/s
    Rotation2d robotAngle = getRotation2d();
    fieldOrientedDrive(
        forwardVelocity * robotAngle.getCos() - strafeVelocity * robotAngle.getSin(),
        forwardVelocity * robotAngle.getSin() + strafeVelocity * robotAngle.getCos(),
        angularVelocity);
  }

  // Drive the robot relative to the field
  public void fieldOrientedDrive(double xVelocity, double yVelocity, double angularVelocity) { // m/s and rad/s
    double speed = Math.hypot(xVelocity, yVelocity);

    if (speed > SwerveDriveConstants.VELOCITY_DEADZONE) {
      driveDirection = Math.atan2(yVelocity, xVelocity);
    }

    speed = accelerationLimiter.calculate(speed);
    xVelocity = speed * Math.cos(driveDirection);
    yVelocity = speed * Math.sin(driveDirection);

    angularVelocity = angularAccelerationLimiter.calculate(angularVelocity);

    Rotation2d robotAngle = getRotation2d();
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, robotAngle);
    SwerveModuleState moduleStates[] = kinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  // Set all modules target speed and directions
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveMath.motorPowerToWheelVelocity(SwerveDriveConstants.MOTOR_POWER_HARD_CAP));
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setState(moduleStates[i]);
    }
  }

  // Get all modules speed and directions
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        swerveModules[0].getModulePosition(),
        swerveModules[1].getModulePosition(),
        swerveModules[2].getModulePosition(),
        swerveModules[3].getModulePosition()
    };
  }

  // Get all modules target speed and directions
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getTargetModuleStates());
  }

  public SwerveModuleState[] getTargetModuleStates() {
    return new SwerveModuleState[] {
        swerveModules[0].getState(),
        swerveModules[1].getState(),
        swerveModules[2].getState(),
        swerveModules[3].getState()
    };
  }

  public SwerveModuleState[] getMeasuredModuleStates() {
    return new SwerveModuleState[] {
        swerveModules[0].getMeasuredState(),
        swerveModules[1].getMeasuredState(),
        swerveModules[2].getMeasuredState(),
        swerveModules[3].getMeasuredState()
    };
  }

  public SwerveModule[] getModules() {
    return swerveModules;
  }

  // Run all modules motors, must be called periodically
  public void driveModules() {
    for (SwerveModule module : swerveModules) {
      module.drive();
    }
  }

  // This creates an "X" pattern with the wheels which makes the robot very hard to move
  public void groundModules() {
    swerveModules[0].setState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    swerveModules[1].setState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    swerveModules[2].setState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    swerveModules[3].setState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
  }

  // Get total voltage through all modules
  public double getVoltage() {
    double totalVoltage = 0.0;
    for (SwerveModule module : swerveModules) {
      totalVoltage += module.getVoltage();
    }
    return totalVoltage;
  }

  // Get total current through all modules
  public double getCurrent() {
    double totalCurrent = 0.0;
    for (SwerveModule module : swerveModules) {
      totalCurrent += module.getCurrent();
    }
    return totalCurrent;
  }

  // Get pose on field
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  // Set pose on field
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  // Stop motors on all modules
  public void stopModules() {
    for (SwerveModule module : swerveModules) {
      module.stop();
    }
  }

  public void selfCheckModules() {
    for (SwerveModule module : swerveModules) {
      module.selfCheck();
    }
  }
}