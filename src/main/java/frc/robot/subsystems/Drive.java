// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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
import frc.robot.Constants;

import com.ctre.phoenix.sensors.CANCoder;

public class Drive extends SubsystemBase {

  private CANSparkMax[] driveSparks = new CANSparkMax[4];
  private CANSparkMax[] steerSparks = new CANSparkMax[4];
  private CANCoder[] canCoders;

  private IMU IMU;

  public Drive(IMU IMU) {
    this.IMU = IMU;

    for (int i = 0; i < 4; i++) {
      driveSparks[i] = new CANSparkMax(Constants.DRIVE_CAN_IDS[i], MotorType.kBrushless);
      steerSparks[i] = new CANSparkMax(Constants.STEER_CAN_IDS[i], MotorType.kBrushless);
      canCoders[i] = new CANCoder(Constants.CANCODER_CAN_IDS[i]);
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

  public void runSpark(double speed) {
    //spark.set(speed);
  }

  public void setIdleMode(CANSparkMax.IdleMode idleMode) {
    //spark.setIdleMode(idleMode);
  }

  public void resetEncoders() {

  }
}
