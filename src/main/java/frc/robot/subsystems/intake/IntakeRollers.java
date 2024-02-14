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
import frc.robot.Constants.AMP.WHEELS;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;



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
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
    
    motor = new CANSparkMax(CAN.CENTERING, MotorType.kBrushless);

    ConfigUtils.configure(List.of(
      () -> motor.restoreFactoryDefaults(),
      () -> { motor.setInverted(false); return true; },
      () -> motor.setIdleMode(IdleMode.kBrake),
      () -> motor.enableVoltageCompensation(12.0),
      () -> motor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, NEO.SAFE_STALL_CURRENT),
      () -> motor.setClosedLoopRampRate(NEO.SAFE_RAMP_RATE),
      () -> motor.burnFlash()
    ));

    detector = new NoteDetector(motor, WHEELS.NOTE_DETECTION_CURRENT);

    String logPath = "intake-wheels/";
    Logger.autoLog(logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> motor.getMotorTemperature());
    Logger.autoLog(logPath + "hasNote",                 () -> detector.hasNote());

    StatusChecks.addCheck("Intake Motor", () -> motor.getFaults() == 0);
  }

  public Command setState(State state) {
    return runOnce(() -> this.state = state);
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
    switch(state) {
      case IN:
        motor.set(Presets.INTAKE.INTAKE_ROLLER_POWER);
        detector.run();
      case OUT:
        motor.set(-Presets.INTAKE.INTAKE_ROLLER_POWER);
        detector.run();
      case OFF:
        motor.set(0);
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
