// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.SHOOTER_PIVOT;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.hardware.MotionControl.PivotController;

public class ShooterPivot extends SubsystemBase {
  private CANSparkMax motor;
  private PivotController controller;
  private boolean isCalibrating = false;

  public ShooterPivot() {    
    motor = new CANSparkMax(CAN.SHOOTER_PIVOT, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, false, IdleMode.kBrake);
    SparkMaxUtil.save(motor);

    controller = new PivotController(
      this,
      motor,
      DIO.SHOOTER_PIVOT,
      SHOOTER_PIVOT.ABSOLUTE_POSITION_OFFSET,
      SHOOTER_PIVOT.PROFILE.kP,
      SHOOTER_PIVOT.GEARING,
      SHOOTER_PIVOT.PROFILE.MAX_ACCELERATION,
      Preferences.SHOOTER_PIVOT.MIN_ANGLE,
      Preferences.SHOOTER_PIVOT.MAX_ANGLE,
      true
    );

    SparkMaxUtil.save(motor);
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
    return Math.abs(getPosition().minus(controller.getTargetAngle()).getRadians()) < SHOOTER_PIVOT.ANGLE_TOLERANCE.getRadians();
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
