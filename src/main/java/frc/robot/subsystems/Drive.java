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

import frc.robot.commands.*;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

  private CANSparkMax leftBank1 = new CANSparkMax(Constants.CAN_LEFT_DRIVE_1, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax leftBank2 = new CANSparkMax(Constants.CAN_LEFT_DRIVE_2, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax rightBank1 = new CANSparkMax(Constants.CAN_RIGHT_DRIVE_1, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax rightBank2 = new CANSparkMax(Constants.CAN_RIGHT_DRIVE_2, CANSparkMax.MotorType.kBrushless);

  private MotorControllerGroup rightBank = new MotorControllerGroup(rightBank1, rightBank2);
  private MotorControllerGroup leftBank = new MotorControllerGroup(leftBank1, leftBank2);

  private DifferentialDrive drive = new DifferentialDrive(leftBank, rightBank);

  private RelativeEncoder leftBankEncoder = leftBank2.getEncoder();
  private RelativeEncoder rightBankEncoder = rightBank1.getEncoder();

  private DifferentialDriveOdometry odometry;
  private Field2d field = new Field2d();

  private IMU IMU;

  public Drive(IMU IMU) {
    this.IMU = IMU;

  }

  @Override
  public void periodic() {
    

    // This method will be called once per scheduler run
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setIdleMode(CANSparkMax.IdleMode idleMode) {
    leftBank1.setIdleMode(idleMode);
    leftBank2.setIdleMode(idleMode);
    rightBank1.setIdleMode(idleMode);
    rightBank2.setIdleMode(idleMode);
  }

  public void resetEncoders() {
    leftBankEncoder.setPosition(0);
    rightBankEncoder.setPosition(0);
  }
}
