// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SHOOTER.PIVOT;
import frc.robot.Presets;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.hardware.MotionControl.PivotController;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

public class ShooterPivot extends SubsystemBase {
  private CANSparkMax motor;
  private PivotController controller;
  private boolean isCalibrating = false;

  public ShooterPivot() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    
    motor = new CANSparkMax(CAN.SHOOTER_PIVOT, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, false, IdleMode.kBrake);
    SparkMaxUtil.save(motor);

    controller = new PivotController(
      motor,
      DIO.SHOOTER_PIVOT,
      PIVOT.ABSOLUTE_POSITION_OFFSET,
      PIVOT.PROFILE.kP,
      PIVOT.GEARBOX_REDUCTION,
      PIVOT.PROFILE.MAX_ACCELERATION,
      Presets.SHOOTER.PIVOT.MIN_ANGLE,
      Presets.SHOOTER.PIVOT.MAX_ANGLE,
      true
    );

    Logger.autoLog(this, "absolutePosition",        () -> controller.getAbsolutePosition().getRadians());
    Logger.autoLog(this, "rawAbsolutePosition",        () -> controller.getRawAbsoluteEncoderValue());
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    if (isCalibrating) return;
    if (RobotState.isDisabled()) {
      controller.setTargetAngle(getPosition());
    }
    // controller.run();
  }

  public Command setTargetAngle(Rotation2d angle) {
    return runOnce(() -> controller.setTargetAngle(angle));
  }

  public Rotation2d getPosition() {
    return controller.getPosition();
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
          log.motor("shooter-pivot")
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
