// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
import edu.wpi.first.wpilibj.XboxController;

import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConfig;
import frc.robot.subsystems.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule[] swerveModules = new SwerveModule[4];
  private AHRS gyro;
  private SwerveDriveKinematics kinematics = getKinematics();
  private SwerveDriveOdometry odometer;

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
        SwerveDriveConfig.STARTING_POSE);

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
    odometer.update(getRotation2d(), getModulePositions());

    selfCheckModules();
    SelfCheck.checkPDPFaults();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Get gyro Rotation2d heading
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(((gyro.getRotation2d().getDegrees() + 180) % 360) - 180);
  }

  // Get gyro degree heading
  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  // Reset gyro heading
  public void zeroHeading() {
    gyro.reset();
    gyro.setAngleAdjustment(SwerveDriveConfig.STARTING_ANGLE_OFFSET);
  }

  // Drive the robot relative to the field
  public void fieldOrientedDrive(double forward, double strafe, double rotation) { // m/s and rad/s
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getRotation2d());
    SwerveModuleState moduleStates[] = kinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  // Drive the robot relative to itself
  public void robotOrientedDrive(double forward, double strafe, double rotation) { // m/s and rad/s
    ChassisSpeeds speeds = new ChassisSpeeds(forward, strafe, rotation);
    SwerveModuleState moduleStates[] = kinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  // Set all modules target speed and directions
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModule.motorPowerToWheelVelocity(SwerveDriveConfig.MOTOR_POWER_HARD_CAP));
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setTargetState(moduleStates[i]);
    }
  }

  // Get all modules speed and directions
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        swerveModules[0].getPosition(),
        swerveModules[1].getPosition(),
        swerveModules[2].getPosition(),
        swerveModules[3].getPosition()
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
    swerveModules[0].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    swerveModules[1].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    swerveModules[2].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    swerveModules[3].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
  }

  // Tune PID values, only used in testing mode
  public void setPID(double[] PID) {
    for (SwerveModule module : swerveModules) {
      module.setPID(PID);
    }
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModules;
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

  public static SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(
        new Translation2d(SwerveDriveConfig.TRACKWIDTH_METERS / 2.0, SwerveDriveConfig.WHEELBASE_METERS / 2.0),
        new Translation2d(SwerveDriveConfig.TRACKWIDTH_METERS / 2.0, -SwerveDriveConfig.WHEELBASE_METERS / 2.0),
        new Translation2d(-SwerveDriveConfig.TRACKWIDTH_METERS / 2.0, SwerveDriveConfig.WHEELBASE_METERS / 2.0),
        new Translation2d(-SwerveDriveConfig.TRACKWIDTH_METERS / 2.0, -SwerveDriveConfig.WHEELBASE_METERS / 2.0));
  }

  // Calculate max angular velocity from max module drive velocity
  public static double wheelVelocityToRotationalVelocity(double maxDriveVelocity) {
    return maxDriveVelocity / Math.hypot( // measured in radians/second
        SwerveDriveConfig.TRACKWIDTH_METERS / 2.0,
        SwerveDriveConfig.WHEELBASE_METERS / 2.0);
  }

  // Calculate max module drive velocity from max angular velocity
  public static double rotationalVelocityToWheelVelocity(double maxAngularVelocity) {
    return maxAngularVelocity * Math.hypot(
        SwerveDriveConfig.TRACKWIDTH_METERS / 2.0,
        SwerveDriveConfig.WHEELBASE_METERS / 2.0);
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