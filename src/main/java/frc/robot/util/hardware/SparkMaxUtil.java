package frc.robot.util.hardware;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NEO;
import frc.robot.Constants.SHOOTER.WHEELS;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

public final class SparkMaxUtil {
  public static void configureAndLog(SubsystemBase subsystem, CANSparkMax motor, boolean inverted, IdleMode idleMode) {
    configureAndLog(subsystem, motor, inverted, idleMode, NEO.SAFE_STALL_CURRENT);
  }

  public static void configureAndLog(SubsystemBase subsystem, CANSparkMax motor, boolean inverted, IdleMode idleMode, int currentLimit) {
    motor.restoreFactoryDefaults();
    motor.setInverted(inverted);
    motor.setIdleMode(idleMode);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(Math.min(currentLimit, NEO.SAFE_STALL_CURRENT), currentLimit);
    motor.setClosedLoopRampRate(NEO.SAFE_RAMP_RATE);
    motor.setOpenLoopRampRate(NEO.SAFE_RAMP_RATE);

    RelativeEncoder encoder = motor.getEncoder();

    String logPath = "/motor" + motor.getDeviceId() + "/";

    Logger.autoLog(subsystem, logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(subsystem, logPath + "voltage",                 () -> motor.getBusVoltage());
    Logger.autoLog(subsystem, logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(subsystem, logPath + "appliedVoltage",          () -> motor.getBusVoltage() * motor.getAppliedOutput());
    Logger.autoLog(subsystem, logPath + "motorTemperature",        () -> motor.getMotorTemperature());
    Logger.autoLog(subsystem, logPath + "position",                () -> encoder.getPosition());
    Logger.autoLog(subsystem, logPath + "velocity",                () -> encoder.getVelocity());
    
    StatusChecks.addCheck(subsystem, logPath + "hasFaults", () -> motor.getFaults() == 0 && motor.getDeviceId() != 0);
    StatusChecks.addCheck(subsystem, logPath + "isConnected",    () -> motor.getDeviceId() != 0);
  }

  public static void configureAndLog550(SubsystemBase subsystem, CANSparkMax motor, boolean inverted, IdleMode idleMode) {
    configureAndLog(subsystem, motor, inverted, idleMode, 10);
  }

  public enum StatusFrames {
    _0_ONLY_FAULTS,
    _1_MOTOR_DATA,
    _2_MOTOR_DATA_AND_POSITION,
  }

  public static void setFrames(CANSparkMax motor, StatusFrames frames) {
    switch (frames) {
      case _0_ONLY_FAULTS:
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        break;
      case _1_MOTOR_DATA:
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        break;
      case _2_MOTOR_DATA_AND_POSITION:
        break;
    }
  }

  public static void configurePID(CANSparkMax motor, double kP, double kI, double kD, double kV, boolean wrap) {
    SparkPIDController pid = motor.getPIDController();
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(kV / 12.0, 0);

    if (wrap) {
      pid.setPositionPIDWrappingEnabled(true);
      pid.setPositionPIDWrappingMinInput(-Math.PI);
      pid.setPositionPIDWrappingMaxInput(Math.PI);
    }
  }

  public static void configureEncoder(CANSparkMax motor, double encoderConversionFactor) {
    RelativeEncoder encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(encoderConversionFactor);
    encoder.setVelocityConversionFactor(encoderConversionFactor / 60.0);
  }

  public static void save(CANSparkMax motor) {
    motor.burnFlash();
  }
}
