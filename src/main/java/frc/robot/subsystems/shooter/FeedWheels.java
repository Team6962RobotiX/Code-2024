// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.SparkMaxUtil;

public class FeedWheels extends SubsystemBase {
  private CANSparkMax motor;
  // private NoteDetector detector;
  private State state = State.OFF;
 
  public static enum State {
    IN,
    OUT,
    OFF,
    SHOOT
  }

  public FeedWheels() {
    motor = new CANSparkMax(CAN.SHOOTER_FEED, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, true, CANSparkMax.IdleMode.kCoast, 80, 80);
    SparkMaxUtil.configureCANStatusFrames(motor, false, false);
    SparkMaxUtil.save(motor);

    // detector = new NoteDetector(motor, Constants.SHOOTER_FEED.GEARING, Constants.SHOOTER_FEED.FREE_TORQUE, false);

  }

  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    if (RobotState.isDisabled()) {
      state = State.OFF;
    }
    switch(state) {
      case OFF:
        motor.set(0);
        break;
      case IN:
        motor.set(Preferences.SHOOTER_FEED.POWER_IN);
        break;
      case SHOOT:
        motor.set(Preferences.SHOOTER_FEED.POWER_SHOOT);
        break;
      case OUT:
        motor.set(-Preferences.SHOOTER_FEED.POWER_IN);
        break;
    }

    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.SHOOTER) motor.stopMotor();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
