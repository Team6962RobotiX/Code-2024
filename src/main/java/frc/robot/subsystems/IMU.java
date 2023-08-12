// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.commands.*;
import frc.robot.Constants;

public class IMU extends SubsystemBase {

  AHRS IMU = new AHRS(SPI.Port.kMXP);;

  public IMU() {
    
  }

  @Override
  public void periodic() {
    // System.out.println(IMU.getRoll());
    // System.out.println(IMU.isCalibrating());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getRoll() {
    return IMU.getRoll();
  }

  public AHRS getIMU() {
    return IMU;
  }

  public Rotation2d getRotation2d() {
    return IMU.getRotation2d();
  }
}
