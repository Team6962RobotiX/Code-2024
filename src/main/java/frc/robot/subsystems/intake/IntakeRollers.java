// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.NoteDetector;
import frc.robot.util.hardware.SparkMaxUtil;

public class IntakeRollers extends SubsystemBase {
  private CANSparkMax motor;
  private State state = State.OFF;
  private NoteDetector detector;

  public static enum State {
    IN,
    OUT,
    OFF
  }

  public IntakeRollers() {    
    motor = new CANSparkMax(CAN.INTAKE, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, false, IdleMode.kBrake);
    SparkMaxUtil.save(motor);

    detector = new NoteDetector(motor, Constants.INTAKE_ROLLERS.GEARING, Constants.INTAKE_ROLLERS.FREE_TORQUE);
  }

  public Command setState(State state) {
    return runOnce(() -> this.state = state);
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
    if (RobotState.isDisabled()) {
      state = State.OFF;
    }
    switch(state) {
      case IN:
        motor.set(-Preferences.INTAKE.INTAKE_ROLLER_POWER);
        break;
      case OUT:
        motor.set(Preferences.INTAKE.INTAKE_ROLLER_POWER);
        break;
      case OFF:
        motor.set(0);
        break;
    }
  }

  public boolean hasNote() {
    return detector.hasNote();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
