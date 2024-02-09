// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.amp;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;



public class AmpWheels extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private AmpState state = AmpState.OFF;
 
  public static enum AmpState {
    IN,
    OUT,
    OFF
  }

  public AmpWheels() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
    
    intakeMotor = new CANSparkMax(CAN.AMP_WHEELS, MotorType.kBrushless);
  }

  public void setState(AmpState newState) {
    state = newState;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;

    switch(state) {
      case OFF:
        intakeMotor.set(0);
        break;
      case IN:
        intakeMotor.set(0.45);
        break;
      case OUT:
        intakeMotor.set(-0.45);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
