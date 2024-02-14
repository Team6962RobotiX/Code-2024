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

public class Amp extends SubsystemBase {
  private AmpPivot pivot;
  private AmpWheels wheels;
 
  public static enum State {
    IN,
    UP,
    OUT,
    OFF
  }

  public Amp() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;
    pivot = new AmpPivot();
    wheels = new AmpWheels();
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;
  }

  public Command setState(State state) {
    switch(state) {
      case IN:
        return Commands.sequence( 
          pivot.setTargetAngle(Presets.AMP.PIVOT.INTAKE_ANGLE),
          Commands.waitUntil(() -> pivot.doneMoving()),
          wheels.setState(AmpWheels.State.IN),
          Commands.waitUntil(() -> hasNote()),
          wheels.setState(AmpWheels.State.OFF)
        );
      case UP:
        return Commands.sequence( 
          pivot.setTargetAngle(Presets.AMP.PIVOT.OUTPUT_ANGLE),
          wheels.setState(AmpWheels.State.OFF)
        );
      case OUT:
        return Commands.sequence( 
          pivot.setTargetAngle(Presets.AMP.PIVOT.OUTPUT_ANGLE),
          Commands.waitUntil(() -> pivot.doneMoving()),
          wheels.setState(AmpWheels.State.OUT),
          Commands.waitSeconds(1.0),
          wheels.setState(AmpWheels.State.OFF)
        );
      case OFF:
        return Commands.sequence( 
          wheels.setState(AmpWheels.State.OFF),
          pivot.setTargetAngle(pivot.getPosition())
        );
    }
    return null;
  }

  public boolean hasNote() {
    return wheels.hasNote();
  }

  public boolean doneMoving() {
    return pivot.doneMoving();
  }
}
