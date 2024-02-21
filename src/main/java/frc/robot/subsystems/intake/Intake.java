// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;



public class Intake extends SubsystemBase {
  private CenteringWheels centeringWheels;
  private IntakeRollers intakeRollers;
 
  public static enum State {
    IN,
    OUT,
  }

  public Intake() {
    intakeRollers = new IntakeRollers();
    centeringWheels = new CenteringWheels();
  }

  public Command setState(State state) {
    switch(state) {
      case IN:
        return Commands.parallel( 
          intakeRollers.setState(IntakeRollers.State.IN),
          centeringWheels.setState(CenteringWheels.State.IN)
        );
      case OUT:
        return Commands.parallel( 
          intakeRollers.setState(IntakeRollers.State.OUT),
          centeringWheels.setState(CenteringWheels.State.OUT)
        );
    }
    return null;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
    
  }

  
  public boolean hasNote() {
    return intakeRollers.hasNote();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
