// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

public class Testing extends SubsystemBase {
  CANCoder absoluteSteerEncoder;

  public Testing() {

    absoluteSteerEncoder = new CANCoder(42);

    CANCoderConfiguration CANCoderConfig = new CANCoderConfiguration();
    CANCoderConfig.magnetOffsetDegrees = SwerveDriveConstants.STEER_ENCODER_OFFSETS[3];
    absoluteSteerEncoder.configAllSettings(CANCoderConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
