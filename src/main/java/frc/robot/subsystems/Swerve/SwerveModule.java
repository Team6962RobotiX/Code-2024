// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

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

  public PIDController steerController;
  public PIDController driveController;

  private SwerveModuleState targetState;

  SwerveModule(CANSparkMax driveMotor, CANSparkMax steerMotor, CANCoder steerEncoder) {
    this.driveMotor = driveMotor;
    this.driveEncoder = driveMotor.getEncoder();
    this.steerMotor = steerMotor;
    this.steerEncoder = steerEncoder;
    steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    steerController = new PIDController(Constants.SWERVE_STEER_KP, Constants.SWERVE_STEER_KI,
        Constants.SWERVE_STEER_KD);
    driveController = new PIDController(Constants.SWERVE_DRIVE_KP, Constants.SWERVE_DRIVE_KI,
        Constants.SWERVE_DRIVE_KD);
    steerController.enableContinuousInput(0, 360);
  }

  public void setState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));

    targetState = state;

    double targetDriveSpeed = state.speedMetersPerSecond / (Constants.SWERVE_WHEEL_DIAMETER * Math.PI);
    double targetSteerAngle = state.angle.getDegrees();

    double drivePower = driveController.calculate(getSpeed(), targetDriveSpeed);
    double steerPower = steerController.calculate(getAngle(), targetSteerAngle);

    driveMotor.set(drivePower);
    steerMotor.set(steerPower);
  }

  public void disableModule() {
    driveMotor.set(0);
    steerMotor.set(0);
  }

  public void setAngle(double setpoint) {
    steerController.reset();
    steerController.setSetpoint(setpoint);
  }

  public double getAngle() {
    return steerEncoder.getAbsolutePosition();
  }

  public double getSpeed() {
    return driveEncoder.getVelocity() / 60 / Constants.SWERVE_GEAR_RATIO
        * Constants.SWERVE_WHEEL_DIAMETER * Math.PI;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAngle()));
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }

}