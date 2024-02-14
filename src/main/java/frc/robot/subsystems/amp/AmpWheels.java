// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.amp;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.subsystems.notes.NoteDetector;
import frc.robot.util.ConfigUtils;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;
import frc.robot.Constants;
import frc.robot.Presets;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;
import frc.robot.Constants.AMP.PIVOT;
import frc.robot.Constants.AMP.WHEELS;



public class AmpWheels extends SubsystemBase {
  private CANSparkMax motor;
  private State state = State.OFF;
  private NoteDetector detector;
 
  public static enum State {
    IN,
    OUT,
    OFF
  }

  public AmpWheels() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;
    motor = new CANSparkMax(CAN.AMP_WHEELS, MotorType.kBrushless);

    ConfigUtils.configure(List.of(
      () -> motor.restoreFactoryDefaults(),
      () -> { motor.setInverted(false); return true; },
      () -> motor.setIdleMode(IdleMode.kBrake),
      () -> motor.enableVoltageCompensation(12.0),
      () -> motor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, PIVOT.PROFILE.CURRENT_LIMIT),
      () -> motor.setClosedLoopRampRate(NEO.SAFE_RAMP_RATE),
      () -> motor.burnFlash()
    ));

    detector = new NoteDetector(motor, WHEELS.NOTE_DETECTION_CURRENT);

    String logPath = "amp-wheels/";
    Logger.autoLog(logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> motor.getMotorTemperature());

    StatusChecks.addCheck("Amp Wheels Motor", () -> motor.getFaults() == 0);
  }

  public void setState(State newState) {
    state = newState;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;

    switch(state) {
      case OFF:
        motor.set(0);
        break;
      case IN:
        motor.set(Presets.AMP.WHEELS.POWER);
        break;
      case OUT:
        motor.set(-Presets.AMP.WHEELS.POWER);
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
