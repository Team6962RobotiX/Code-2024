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
import frc.robot.subsystems.notes.NoteDetector;
import frc.robot.util.ConfigUtils;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;
import frc.robot.Constants;
import frc.robot.Presets;
import frc.robot.Constants.AMP.PIVOT;
import frc.robot.Constants.AMP.WHEELS;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;



public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private CANSparkMax centeringMotor;
  private NoteDetector detector;
 
  public static enum State {
    IN,
    OUT,
    OFF
  }

  public Intake() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
    
    intakeMotor = new CANSparkMax(CAN.INTAKE, MotorType.kBrushless);
    centeringMotor = new CANSparkMax(CAN.CENTERING, MotorType.kBrushless);

    ConfigUtils.configure(List.of(
      () -> intakeMotor.restoreFactoryDefaults(),
      () -> { intakeMotor.setInverted(false); return true; },
      () -> intakeMotor.setIdleMode(IdleMode.kBrake),
      () -> intakeMotor.enableVoltageCompensation(12.0),
      () -> intakeMotor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, PIVOT.PROFILE.CURRENT_LIMIT),
      () -> intakeMotor.setClosedLoopRampRate(NEO.SAFE_RAMP_RATE),
      () -> intakeMotor.burnFlash(),

      () -> centeringMotor.restoreFactoryDefaults(),
      () -> { centeringMotor.setInverted(false); return true; },
      () -> centeringMotor.setIdleMode(IdleMode.kBrake),
      () -> centeringMotor.enableVoltageCompensation(12.0),
      () -> centeringMotor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, PIVOT.PROFILE.CURRENT_LIMIT),
      () -> centeringMotor.setClosedLoopRampRate(NEO.SAFE_RAMP_RATE),
      () -> centeringMotor.burnFlash()
    ));

    detector = new NoteDetector(intakeMotor, WHEELS.NOTE_DETECTION_CURRENT);

    String logPath = "intake-wheels/";
    Logger.autoLog(logPath + "current",                 () -> intakeMotor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> intakeMotor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> intakeMotor.getMotorTemperature());
    Logger.autoLog(logPath + "hasNote",                 () -> detector.hasNote());

    logPath = "intake-centering-wheels/";
    Logger.autoLog(logPath + "current",                 () -> centeringMotor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> centeringMotor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> centeringMotor.getMotorTemperature());

    StatusChecks.addCheck("Intake Motor", () -> intakeMotor.getFaults() == 0);
    StatusChecks.addCheck("Intake Centering Motor", () -> centeringMotor.getFaults() == 0);
  }

  public Command setState(State state) {
    switch(state) {
      case IN:
        return Commands.sequence( 
          runOnce(() -> intakeMotor.set(-Presets.INTAKE.INTAKE_ROLLER_POWER)),
          runOnce(() -> centeringMotor.set(Presets.INTAKE.CENTERING_WHEEL_POWER))
        );
      case OUT:
        return Commands.sequence( 
          runOnce(() -> intakeMotor.set(Presets.INTAKE.INTAKE_ROLLER_POWER)),
          runOnce(() -> centeringMotor.set(-Presets.INTAKE.CENTERING_WHEEL_POWER))
        );
      case OFF:
        return Commands.sequence( 
          runOnce(() -> intakeMotor.set(0.0)),
          runOnce(() -> centeringMotor.set(0.0))
        );
    }
    return null;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
  }

  public boolean hasNote() {
    return detector.hasNote();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
