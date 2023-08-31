// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class Testing extends SubsystemBase {
  CANCoder absoluteSteerEncoder;

  public Testing() {
    int id = 0;

    absoluteSteerEncoder = new CANCoder(CAN.SWERVE_STEER_CANCODER[id], "rio");

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
  }

  @Override
  public void periodic() {
    System.out.println(absoluteSteerEncoder.getAbsolutePosition());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
