// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;
import frc.robot.Constants.SHOOTER.WHEELS;
import frc.robot.util.ConfigUtils;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;

public class ShooterWheels extends SubsystemBase {
  private double targetVelocity = 0.0;
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkPIDController pid;
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(WHEELS.PROFILE.kS, WHEELS.PROFILE.kV, WHEELS.PROFILE.kA);
  private boolean isCalibrating = false;

  public ShooterWheels() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;

    motor = new CANSparkMax(CAN.SHOOTER_WHEELS_BOTTOM, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pid = motor.getPIDController();
    

    ConfigUtils.configure(List.of(
      () -> motor.restoreFactoryDefaults(),
      () -> { motor.setInverted(false); return true; },
      () -> motor.setIdleMode(IdleMode.kCoast),
      () -> motor.enableVoltageCompensation(12.0),
      () -> motor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, WHEELS.PROFILE.CURRENT_LIMIT),
      () -> motor.setClosedLoopRampRate(WHEELS.PROFILE.RAMP_RATE),
      () -> encoder.setPositionConversionFactor(WHEELS.ENCODER_CONVERSION_FACTOR),
      () -> encoder.setVelocityConversionFactor(WHEELS.ENCODER_CONVERSION_FACTOR / 60.0),
      () -> pid.setP(WHEELS.PROFILE.kP, 0),
      () -> pid.setI(WHEELS.PROFILE.kI, 0),
      () -> pid.setD(WHEELS.PROFILE.kD, 0),
      () -> pid.setFF(WHEELS.PROFILE.kV / 12.0, 0),
      () -> motor.burnFlash()
    ));

    String logPath = "shooter-wheels/";
    Logger.autoLog(logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> motor.getMotorTemperature());
    Logger.autoLog(logPath + "position",                () -> encoder.getPosition());
    Logger.autoLog(logPath + "velocity",                () -> getVelocity());
    
    StatusChecks.addCheck("Shooter Wheels Motor", () -> motor.getFaults() == 0);
  }

  public void setTargetVelocity(double angularVelocity) {
    targetVelocity = angularVelocity;
  }

  public double getVelocity() {
    return WHEELS.TARGET_SPEED;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    if (isCalibrating) return;

    pid.setReference(
      targetVelocity,
      ControlType.kVelocity,
      0,
      feedforward.calculate(targetVelocity, 0.0),
      ArbFFUnits.kVoltage
    );
  }

  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  }

  public Command calibrate() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          motor.setVoltage(volts.in(Volts));
        },
        log -> {
          log.motor("shooter-wheels")
            .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
            .angularPosition(Radians.of(encoder.getPosition()))
            .angularVelocity(RadiansPerSecond.of(encoder.getVelocity()));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> isCalibrating = true),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> isCalibrating = false)
    );
  }
}
