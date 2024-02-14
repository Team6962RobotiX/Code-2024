// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.subsystems.amp.AmpWheels;
import frc.robot.util.ConfigUtils;
import frc.robot.util.NoteDetector;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;
import frc.robot.Constants;
import frc.robot.Presets;
import frc.robot.Constants.AMP.PIVOT;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;



public class Intake extends SubsystemBase {
  private CenteringWheels centeringWheels;
  private IntakeRollers intakeRollers;
 
  public static enum State {
    IN,
    OUT,
    OFF
  }

  public Intake() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;

    intakeRollers = new IntakeRollers();
    centeringWheels = new CenteringWheels();
  }

  public Command setState(State state) {
    switch(state) {
      case IN:
        return Commands.sequence( 
          intakeRollers.setState(IntakeRollers.State.IN),
          centeringWheels.setState(CenteringWheels.State.IN)
        );
      case OUT:
        return Commands.sequence( 
          intakeRollers.setState(IntakeRollers.State.OUT),
          centeringWheels.setState(CenteringWheels.State.OUT)
        );
      case OFF:
        return Commands.sequence( 
          intakeRollers.setState(IntakeRollers.State.OFF),
          centeringWheels.setState(CenteringWheels.State.OFF)
        );
    }
    return null;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
  }

  public boolean hasJustReleaseddNote() {
    return intakeRollers.hasJustReleaseddNote();
  }

  public boolean hasJustReceivedNote() {
    return intakeRollers.hasJustReceivedNote();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
