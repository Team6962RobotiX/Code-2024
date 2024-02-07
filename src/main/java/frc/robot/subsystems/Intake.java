// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;



public class Intake extends SubsystemBase {
  private CANSparkMax motor;
  private IntakeState state = IntakeState.OFF;
 
  public static enum IntakeState {
    FORWARD,
    REVERSE,
    OFF
  }

  public Intake() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
    
    motor = new CANSparkMax(CAN.INTAKE, MotorType.kBrushless);
  }

  public void setState(IntakeState newState) {
    state = newState;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;

    switch(state) {
      case OFF:
        motor.set(0);
        break;
      case FORWARD:
        motor.set(1.0);
        break;
      case REVERSE:
        motor.set(-1.0);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
