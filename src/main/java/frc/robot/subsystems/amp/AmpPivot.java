// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.amp;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AMP.PIVOT;
import frc.robot.Presets;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;
import frc.robot.util.ConfigUtils;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;
import frc.robot.util.MotionControl.PivotController;

public class AmpPivot extends SubsystemBase {
  private CANSparkMax motor;
  private PivotController controller;
  private boolean isCalibrating = false;

  public AmpPivot() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;
    
    motor = new CANSparkMax(CAN.AMP_PIVOT, MotorType.kBrushless);
    
    // Configure everything robustly
    ConfigUtils.configure(List.of(
      () -> motor.restoreFactoryDefaults(),
      () -> { motor.setInverted(true); return true; },
      () -> motor.setIdleMode(IdleMode.kBrake),
      () -> motor.enableVoltageCompensation(12.0),
      () -> motor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, PIVOT.PROFILE.CURRENT_LIMIT),
      () -> motor.setClosedLoopRampRate(PIVOT.PROFILE.RAMP_RATE)
    ));

    controller = new PivotController(
      motor,
      DIO.AMP_PIVOT,
      PIVOT.ABSOLUTE_POSITION_OFFSET,
      PIVOT.PROFILE.kP,
      PIVOT.GEARBOX_REDUCTION,
      PIVOT.PROFILE.MAX_ACCELERATION,
      Presets.AMP.PIVOT.MIN_ANGLE,
      Presets.AMP.PIVOT.MAX_ANGLE
    );

    String logPath = "amp-pivot/";
    Logger.autoLog(logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> motor.getMotorTemperature());
    Logger.autoLog(logPath + "position",                () -> controller.getPosition().getRadians());
    Logger.autoLog(logPath + "absolutePosition",        () -> controller.getAbsolutePosition().getRadians());

    StatusChecks.addCheck("Amp Pivot Motor", () -> motor.getFaults() == 0);
    StatusChecks.addCheck("Amp Pivot Absolute Encoder", () -> controller.isAbsoluteEncoderConnected());
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;
    if (isCalibrating) return;
    controller.run();
  }

  public Rotation2d getPosition() {
    return controller.getPosition();
  }

  public void setTargetAngle(Rotation2d angle) {
    controller.setTargetAngle(angle);    
  }

  public boolean doneMoving() {
    return Math.abs(getPosition().minus(controller.getTargetAngle()).getRadians()) < PIVOT.ANGLE_TOLERANCE.getRadians();
  }

  public Command calibrate() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          motor.setVoltage(volts.in(Volts));
        },
        log -> {
          log.motor("amp-pivot")
            .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
            .angularPosition(Radians.of(controller.getPosition().getRadians()))
            .angularVelocity(RadiansPerSecond.of(controller.getVelocity().getRadians()));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> isCalibrating = true),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(controller::isPastLimit),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(controller::isPastLimit),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(controller::isPastLimit),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(controller::isPastLimit),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> isCalibrating = false)
    );
  }
}
