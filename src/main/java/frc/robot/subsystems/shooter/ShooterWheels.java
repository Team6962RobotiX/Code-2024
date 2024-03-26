// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.SHOOTER_WHEELS;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.software.Logging.Logger;

public class ShooterWheels extends SubsystemBase {
  private CANSparkMax motor, motorFollower;
  private RelativeEncoder encoder;
  private boolean isCalibrating = false;
  private State state = State.OFF;
  private double speed = Constants.SHOOTER_WHEELS.MAX_WHEEL_SPEED;
  
  public enum State {
    SPIN_UP,
    OFF,
  }

  public ShooterWheels() {
    motor = new CANSparkMax(CAN.SHOOTER_WHEELS_BOTTOM, MotorType.kBrushless);
    motorFollower = new CANSparkMax(CAN.SHOOTER_WHEELS_TOP, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motorFollower, false, IdleMode.kCoast);
    
    encoder = motor.getEncoder();

    SparkMaxUtil.configureAndLog(this, motor, false, CANSparkMax.IdleMode.kCoast);
    SparkMaxUtil.configureEncoder(motor, SHOOTER_WHEELS.ENCODER_CONVERSION_FACTOR);
    SparkMaxUtil.configureEncoder(motorFollower, SHOOTER_WHEELS.ENCODER_CONVERSION_FACTOR);
    // SparkMaxUtil.configurePID(this, motor, SHOOTER_WHEELS.PROFILE.kP, SHOOTER_WHEELS.PROFILE.kI, SHOOTER_WHEELS.PROFILE.kD, SHOOTER_WHEELS.PROFILE.kV, false);
    SparkMaxUtil.configureCANStatusFrames(motor, true, false);
    SparkMaxUtil.configureCANStatusFrames(motorFollower, true, false);
    SparkMaxUtil.save(motor);

    motorFollower.follow(motor, true);
    SparkMaxUtil.save(motorFollower);
  }

  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }

  public double getVelocity() {
    if (Robot.isSimulation()) return state == State.SPIN_UP ? ShooterMath.calcShooterWheelVelocity(speed) : 0.0;
    if (RobotState.isAutonomous()) return ShooterMath.calcShooterWheelVelocity(13.0);
    return Math.round(encoder.getVelocity() / 20.0) * 20.0;
  }

  public State getState() {
    return state;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    if (isCalibrating) return;

    if (RobotState.isDisabled()) {
      state = State.OFF;
    }

    if (RobotState.isAutonomous()) {
      state = State.SPIN_UP;
      speed = SHOOTER_WHEELS.MAX_WHEEL_SPEED;
    }

    Logger.log("encoder.getVelocity();", getVelocity());

    // System.out.println(speed);
    // System.out.println(ShooterMath.calcProjectileVelocity(ShooterMath.calcShooterWheelVelocity(speed)));

    switch(state) {
      case SPIN_UP:
        // System.out.println(speed);
        motor.set((speed / SHOOTER_WHEELS.MAX_WHEEL_SPEED) / 0.8888349515 / 1.0328467153);
        break;
      case OFF:
        motor.set(0.0);
        break;
    }

    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.SHOOTER) {
      motor.stopMotor();
      motorFollower.stopMotor();
    }
  }

  public Command setTargetWheelSpeedCommand(Supplier<Double> speed) {
    return Commands.runEnd(
      () -> this.speed = speed.get(),
      () -> this.speed = Constants.SHOOTER_WHEELS.MAX_WHEEL_SPEED
    );
  }

  public Command setTargetExitVelocityCommand(Supplier<Double> exitVelocity) {
    return Commands.runEnd(
      () -> this.speed = ShooterMath.calcShooterWheelVelocity(Math.round(exitVelocity.get() * 10.0) / 10.0),
      () -> this.speed = Constants.SHOOTER_WHEELS.MAX_WHEEL_SPEED
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
