// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveMath;
import frc.robot.utils.SelfCheck;
import frc.robot.utils.SwerveModule;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule[] swerveModules = new SwerveModule[4];
  private AHRS gyro;
  private SwerveDriveKinematics kinematics = SwerveMath.getKinematics();
  private SwerveDriveOdometry odometer;

  public SwerveDrive() {
    try {
      gyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      SelfCheck.warn("Error instantiating navX-MXP:  " + ex.getMessage());
    }

    for (int i = 0; i < 4; i++) swerveModules[i] = new SwerveModule(i);

    odometer = new SwerveDriveOdometry(
        kinematics,
        getRotation2d(),
        getModulePositions(),
        SwerveDriveConstants.STARTING_POSE);

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {}
    }).start();
  }

  @Override
  public void periodic() {
    odometer.update(getRotation2d(), getModulePositions());
    selfCheckModules();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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
    Rotation2d robotAngle = getRotation2d();
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, robotAngle);
    SwerveModuleState moduleStates[] = kinematics.toSwerveModuleStates(speeds);
    boolean moving = false;
    for (SwerveModuleState moduleState : moduleStates) {
      if (moduleState.speedMetersPerSecond > SwerveDriveConstants.VELOCITY_DEADBAND) {
        moving = true;
        break;
      }
    }
    // for (SwerveModule module : swerveModules) {
    //   if (module.getVelocity() > SwerveDriveConstants.VELOCITY_DEADBAND) {
    //     moving = true;
    //     break;
    //   }
    // }
    if (moving) {
      driveModules(moduleStates);
    } else {
      groundModules();
    }
  }

  // Set all modules target speed and directions
  public void driveModules(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveMath.motorPowerToWheelVelocity(SwerveDriveConstants.MOTOR_POWER_HARD_CAP));
    for (int i = 0; i < 4; i++) swerveModules[i].drive(moduleStates[i]);
  }

  // This creates an "X" pattern with the wheels which makes the robot very hard to move
  public void groundModules() {
    swerveModules[0].drive(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    swerveModules[1].drive(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    swerveModules[2].drive(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    swerveModules[3].drive(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
  }

  // Set pose on field
  public void resetPose(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  // Stop motors on all modules
  public void stopModules() {
    for (SwerveModule module : swerveModules) module.stop();
  }

  public void selfCheckModules() {
    for (SwerveModule module : swerveModules) module.selfCheck();
  }

  // Get all modules target speed and directions
  public ChassisSpeeds getTargetChassisSpeeds() {
    return kinematics.toChassisSpeeds(getTargetModuleStates());
  }

  // Get all modules target speed and directions
  public ChassisSpeeds getMeasuredChassisSpeeds() {
    return kinematics.toChassisSpeeds(getMeasuredModuleStates());
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

  public SwerveModuleState[] getTargetModuleStates() {
    return new SwerveModuleState[] {
        swerveModules[0].getTargetState(),
        swerveModules[1].getTargetState(),
        swerveModules[2].getTargetState(),
        swerveModules[3].getTargetState()
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

  // Get total current through all modules
  public double getCurrent() {
    double totalCurrent = 0.0;
    for (SwerveModule module : swerveModules) totalCurrent += module.getCurrent();
    return totalCurrent;
  }

  // Get gyro Rotation2d heading
  public AHRS getGyro() {
    return gyro;
  }

  // Reset gyro heading
  public void zeroHeading() {
    gyro.reset();
    gyro.setAngleAdjustment(SwerveDriveConstants.STARTING_ANGLE_OFFSET);
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
    return ((((getRotation2d().getDegrees() + 180.0) % 360.0) + 360.0) % 360.0) - 180.0;
  }

  // Get pose on field
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }
}