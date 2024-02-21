// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.SparkMaxUtil;

public class CenteringWheels extends SubsystemBase {
  private CANSparkMax motor;
  private State state = State.OFF;

  public static enum State {
    IN,
    OUT,
    OFF
  }

  public CenteringWheels() {
    motor = new CANSparkMax(CAN.CENTERING, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, false, IdleMode.kBrake);
    SparkMaxUtil.save(motor);
  }

  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
    if (RobotState.isDisabled()) {
      state = State.OFF;
    }
    switch(state) {
      case IN:
        motor.set(Preferences.INTAKE.CENTERING_WHEEL_POWER);
        break;
      case OUT:
        motor.set(-Preferences.INTAKE.CENTERING_WHEEL_POWER);
        break;
      case OFF:
        motor.set(0);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
