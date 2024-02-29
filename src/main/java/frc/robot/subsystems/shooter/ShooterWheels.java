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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.SHOOTER_WHEELS;
import frc.robot.util.hardware.SparkMaxUtil;

public class ShooterWheels extends SubsystemBase {
  private double targetVelocity = 0.0;
  private CANSparkMax motor, motorFollower;
  private RelativeEncoder encoder;
  private SparkPIDController pid;
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SHOOTER_WHEELS.PROFILE.kS, SHOOTER_WHEELS.PROFILE.kV, SHOOTER_WHEELS.PROFILE.kA);
  private boolean isCalibrating = false;

  public ShooterWheels() {
    motor = new CANSparkMax(CAN.SHOOTER_WHEELS_BOTTOM, MotorType.kBrushless);
    motorFollower = new CANSparkMax(CAN.SHOOTER_WHEELS_TOP, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motorFollower, false, IdleMode.kCoast);
    
    encoder = motor.getEncoder();
    pid = motor.getPIDController();

    SparkMaxUtil.configureAndLog(this, motor, false, CANSparkMax.IdleMode.kCoast);
    SparkMaxUtil.configureEncoder(motor, SHOOTER_WHEELS.ENCODER_CONVERSION_FACTOR);
    SparkMaxUtil.configurePID(this, motor, SHOOTER_WHEELS.PROFILE.kP, SHOOTER_WHEELS.PROFILE.kI, SHOOTER_WHEELS.PROFILE.kD, SHOOTER_WHEELS.PROFILE.kV, false);
    SparkMaxUtil.configureCANStatusFrames(motor, true, false);
    SparkMaxUtil.configureCANStatusFrames(motorFollower, true, false);
    SparkMaxUtil.save(motor);

    motorFollower.follow(motor, true);
    SparkMaxUtil.save(motorFollower);
  }

  public Command setTargetVelocity(double angularVelocity) {
    return runEnd(
      () -> targetVelocity = angularVelocity,
      () -> targetVelocity = 0.0
    );
  }

  public double getVelocity() {
    if (Robot.isSimulation()) return targetVelocity;
    return encoder.getVelocity();
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    if (isCalibrating) return;
    if (RobotState.isAutonomous()) {
      targetVelocity = Preferences.SHOOTER_WHEELS.TARGET_SPEED;
    }
    if (RobotState.isDisabled()) {
      setTargetVelocity(0.0);
    }
    if (targetVelocity > 0) {
      motor.set(1.0);
      return;
    }
    motor.set(0.0);
    // pid.setReference(
    //   targetVelocity,
    //   ControlType.kVelocity,
    //   0
    // );
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
