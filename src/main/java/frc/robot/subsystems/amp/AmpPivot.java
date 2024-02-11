// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.amp;

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
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AMP.PIVOT;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;
import frc.robot.util.ConfigUtils;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;

public class AmpPivot extends SubsystemBase {
  private Rotation2d targetAngle = Rotation2d.fromDegrees(0.0);
  private ArmFeedforward feedforward = new ArmFeedforward(0.0, PIVOT.PROFILE.kG, 0.0, 0.0);
  private SparkPIDController pid;
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private DutyCycleEncoder absoluteEncoder;
  private boolean isCalibrating = false;

  public AmpPivot() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;
    
    motor = new CANSparkMax(CAN.AMP_PIVOT, MotorType.kBrushless);
    pid = motor.getPIDController();
    encoder = motor.getEncoder();
    absoluteEncoder = new DutyCycleEncoder(DIO.AMP_PIVOT);
    
    ConfigUtils.configure(List.of(
      () -> motor.restoreFactoryDefaults(),
      () -> { motor.setInverted(true); return true; },
      () -> motor.setIdleMode(IdleMode.kBrake),
      () -> motor.enableVoltageCompensation(12.0),
      () -> motor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, PIVOT.PROFILE.CURRENT_LIMIT),
      () -> motor.setClosedLoopRampRate(PIVOT.PROFILE.RAMP_RATE),
      () -> encoder.setPositionConversionFactor(PIVOT.ENCODER_CONVERSION_FACTOR),
      () -> encoder.setVelocityConversionFactor(PIVOT.ENCODER_CONVERSION_FACTOR / 60.0),
      () -> pid.setP(PIVOT.PROFILE.kP, 0),
      () -> pid.setI(PIVOT.PROFILE.kI, 0),
      () -> pid.setD(PIVOT.PROFILE.kD, 0),
      () -> pid.setFF(PIVOT.PROFILE.kV / 12.0, 0),
      () -> pid.setPositionPIDWrappingEnabled(true),
      () -> pid.setPositionPIDWrappingMinInput(-Math.PI),
      () -> pid.setPositionPIDWrappingMaxInput(Math.PI),
      () -> motor.burnFlash()
    ));

    // pid.setOutputRange(-1.0, 1.0);
    // pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    String logPath = "amp-pivot/";
    Logger.autoLog(logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> motor.getMotorTemperature());
    Logger.autoLog(logPath + "position",                () -> encoder.getPosition());
    Logger.autoLog(logPath + "velocity",                () -> encoder.getVelocity());

    StatusChecks.addCheck("Amp Pivot Motor", () -> motor.getFaults() == 0);
  }

  public void setTargetAngle(Rotation2d angle) {
    if (targetAngle.getRadians() > PIVOT.MAX_ANGLE.getRadians()) {
      targetAngle = PIVOT.MAX_ANGLE;
    }
    if (targetAngle.getRadians() < PIVOT.MIN_ANGLE.getRadians()) {
      targetAngle = PIVOT.MIN_ANGLE;
    }
    targetAngle = angle;
  }

  public boolean isPastLimit() {
    return getMeasuredAngle().getRadians() > PIVOT.MAX_ANGLE.getRadians() || getMeasuredAngle().getRadians() < PIVOT.MIN_ANGLE.getRadians();
  }

  public Rotation2d getMeasuredAngle() {
    return Rotation2d.fromRadians(encoder.getPosition());
  }

  public double getMeasuredVelocity() {
    return encoder.getVelocity();
  }

  public double getAbsoluteAngle() {
    double absoluteAngle = (absoluteEncoder.getAbsolutePosition() + PIVOT.ABSOLUTE_POSITION_OFFSET);
    while (absoluteAngle < 0) absoluteAngle++;
    absoluteAngle %= 1.0;
    
    absoluteAngle *= Math.PI * 2.0;
    if (absoluteAngle > Math.PI) {
      absoluteAngle -= Math.PI * 2.0;
    }

    return absoluteAngle;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;
    if (isCalibrating) return;
    if (targetAngle == null) return;

    pid.setReference(
      targetAngle.getRadians(),
      ControlType.kPosition,
      0
    );

    if (Math.abs(encoder.getVelocity()) < 0.001) {
      encoder.setPosition(getAbsoluteAngle());
    }

    System.out.println("absolute: " + getAbsoluteAngle());
    System.out.println("relative: " + encoder.getPosition());
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
          log.motor("amp-pivot")
            .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
            .angularPosition(Radians.of(encoder.getPosition()))
            .angularVelocity(RadiansPerSecond.of(encoder.getVelocity()));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> isCalibrating = true),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(this::isPastLimit),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(this::isPastLimit),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(this::isPastLimit),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(this::isPastLimit),
      Commands.runOnce(() -> motor.stopMotor()),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> isCalibrating = false)
    );
  }
}
