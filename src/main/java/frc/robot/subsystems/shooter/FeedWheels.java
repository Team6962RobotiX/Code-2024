// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;



public class FeedWheels extends SubsystemBase {
  private CANSparkMax feedMotor;
  private ShooterState state = ShooterState.OFF;
 
  public static enum ShooterState {
    FORWARD,
    REVERSE,
    OFF
  }

  public FeedWheels() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    
    feedMotor = new CANSparkMax(CAN.SHOOTER_FEED, MotorType.kBrushless);
  }

  public void setState(ShooterState newState) {
    state = newState;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;

    switch(state) {
      case OFF:
        feedMotor.set(0);
        break;
      case FORWARD:
        feedMotor.set(0.3);
        break;
      case REVERSE:
        feedMotor.set(-0.3);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
