// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.util.hardware.NoteDetector;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;
import frc.robot.Constants;
import frc.robot.Presets;
import frc.robot.Constants.AMP.PIVOT;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.NEO;

public class FeedWheels extends SubsystemBase {
  private CANSparkMax motor;
  private NoteDetector detector;
  private State state = State.OFF;
 
  public static enum State {
    IN,
    OUT,
    OFF
  }

  public FeedWheels() {
    motor = new CANSparkMax(CAN.SHOOTER_FEED, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, false, IdleMode.kBrake);
    SparkMaxUtil.save(motor);

    detector = new NoteDetector(motor);

    Logger.autoLog(this, "hasJustReleaseddNote",    () -> detector.hasJustReleaseddNote());
    Logger.autoLog(this, "hasJustReceivedNote",     () -> detector.hasJustReceivedNote());
  }

  public Command setState(State state) {
    return runOnce(() -> this.state = state);
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
        motor.set(Presets.SHOOTER.FEED.POWER);
        break;
      case OUT:
        motor.set(-Presets.SHOOTER.FEED.POWER);
        break;
    }
  }

  public boolean hasJustReleaseddNote() {
    return detector.hasJustReleaseddNote();
  }

  public boolean hasJustReceivedNote() {
    return detector.hasJustReceivedNote();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
