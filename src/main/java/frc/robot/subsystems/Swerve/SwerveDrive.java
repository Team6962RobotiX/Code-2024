// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

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
      new Translation2d(Constants.SWERVE_MODULE_SPACING_X / 2, -Constants.SWERVE_MODULE_SPACING_Y / 2),
      new Translation2d(Constants.SWERVE_MODULE_SPACING_X / 2, Constants.SWERVE_MODULE_SPACING_Y / 2),
      new Translation2d(-Constants.SWERVE_MODULE_SPACING_X / 2, Constants.SWERVE_MODULE_SPACING_Y / 2),
      new Translation2d(-Constants.SWERVE_MODULE_SPACING_X / 2, -Constants.SWERVE_MODULE_SPACING_Y / 2));

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  public SwerveDrive() {
    gyro.reset();
    gyro.setAngleAdjustment(Constants.STARTING_ANGLE_OFFSET);

    for (int i = 0; i < 4; i++) {
      swerveModules[i] = new SwerveModule(
          new CANSparkMax(Constants.DRIVE_CAN_IDS[i], MotorType.kBrushless),
          new CANSparkMax(Constants.STEER_CAN_IDS[i], MotorType.kBrushless),
          new CANCoder(Constants.CANCODER_CAN_IDS[i]));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public void fieldOrientedDrive(double forward, double strafe, double rotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation,
        Rotation2d.fromDegrees(getHeading()));
    driveModules(speeds);
  }

  public void robotOrientedDrive(double forward, double strafe, double rotation) {
    ChassisSpeeds speeds = new ChassisSpeeds(forward, strafe, rotation);
    driveModules(speeds);
  }

  private void driveModules(ChassisSpeeds speeds) {
    SwerveModuleState moduleStates[] = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_DRIVE_VELOCITY);
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setState(moduleStates[i]);
    }
  }

  public void groundModules() {
    // This will create a "X" pattern with the modules which will make the robot very difficult to rotate or move
    swerveModules[0].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    swerveModules[1].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    swerveModules[2].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    swerveModules[3].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void disableModules() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].disableModule();
    }
  }

}