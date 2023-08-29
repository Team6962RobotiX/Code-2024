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

import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.subsystems.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule[] swerveModules = new SwerveModule[4];

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private SwerveDriveOdometry odometry;
  private Pose2d pose;

  public SwerveDrive() {
    gyro.reset();
    gyro.setAngleAdjustment(Constants.STARTING_ANGLE_OFFSET);

    for (int i = 0; i < 4; i++) {
      swerveModules[i] = new SwerveModule(
          new CANSparkMax(Constants.CAN_SWERVE_DRIVE[i], MotorType.kBrushless),
          new CANSparkMax(Constants.CAN_SWERVE_STEER[i], MotorType.kBrushless),
          new CANCoder(Constants.CAN_SWERVE_STEER_ENCODER[i]),
          Constants.SWERVE_MODULE_NAMES[i]);
    }

    odometry = new SwerveDriveOdometry(
        kinematics, gyro.getRotation2d(),
        getModulePositions(), Constants.STARTING_POSE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveModules();
    pose = odometry.update(gyro.getRotation2d(),
        getModulePositions());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public void fieldOrientedDrive(double forward, double strafe, double rotation) { // m/s and rev/s
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation * 2.0 * Math.PI,
        Rotation2d.fromDegrees(getHeading()));
    setModuleStates(speeds);
  }

  public void robotOrientedDrive(double forward, double strafe, double rotation) { // m/s and rev/s
    ChassisSpeeds speeds = new ChassisSpeeds(forward, strafe, rotation * 2.0 * Math.PI);
    setModuleStates(speeds);
  }

  private void setModuleStates(ChassisSpeeds speeds) {
    SwerveModuleState moduleStates[] = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SWERVE_MAX_DRIVE_VELOCITY);
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setTargetState(moduleStates[i]);
    }
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        swerveModules[0].getPosition(),
        swerveModules[1].getPosition(),
        swerveModules[2].getPosition(),
        swerveModules[3].getPosition()
    };
  }

  private void driveModules() {
    for (SwerveModule module : swerveModules) {
      module.drive();
    }
  }

  public void groundModules() {
    // This will create a "X" pattern with the modules which will make the robot very difficult to rotate or move
    swerveModules[0].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    swerveModules[1].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    swerveModules[2].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    swerveModules[3].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
  }

  public void setPID(double[] PID) {
    for (SwerveModule module : swerveModules) {
      module.setPID(PID);
    }
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModules;
  }

  public double getVoltage() {
    double totalVoltage = 0.0;
    for (SwerveModule module : swerveModules) {
      totalVoltage += module.getVoltage();
    }
    return totalVoltage;
  }

  public double getCurrent() {
    double totalCurrent = 0.0;
    for (SwerveModule module : swerveModules) {
      totalCurrent += module.getCurrent();
    }
    return totalCurrent;
  }
}