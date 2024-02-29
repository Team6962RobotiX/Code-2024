package frc.robot.util.hardware;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.LOGGING;
import frc.robot.Constants.Constants.NEO;
import frc.robot.util.TunableNumber;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

public final class SparkMaxUtil {
  public static void configureAndLog(SubsystemBase subsystem, CANSparkMax motor, boolean inverted, CANSparkMax.IdleMode idleMode) {
    configureAndLog(subsystem, motor, inverted, idleMode, NEO.SAFE_FREE_CURRENT, NEO.SAFE_STALL_CURRENT);
  }

  public static void configureAndLog(SubsystemBase subsystem, CANSparkMax motor, boolean inverted, CANSparkMax.IdleMode idleMode, int freeCurrentLimit, int stallCurrentLimit) {
    configure(() -> motor.restoreFactoryDefaults(), motor);
    configure(() -> motor.setIdleMode(idleMode), motor);
    configure(() -> motor.enableVoltageCompensation(12.0), motor);
    configure(() -> motor.setSmartCurrentLimit(Math.min(stallCurrentLimit, NEO.SAFE_STALL_CURRENT), Math.min(freeCurrentLimit, NEO.SAFE_FREE_CURRENT)), motor);
    configure(() -> motor.setClosedLoopRampRate(NEO.SAFE_RAMP_RATE), motor);
    configure(() -> motor.setOpenLoopRampRate(NEO.SAFE_RAMP_RATE), motor);
    motor.setInverted(inverted);

    configureCANStatusFrames(motor, 10, 20, 20, 500, 500, 500, 500);

    RelativeEncoder encoder = motor.getEncoder();
    
    String logPath = "motor" + motor.getDeviceId() + "/";

    Logger.autoLog(subsystem, logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(subsystem, logPath + "voltage",                 () -> motor.getBusVoltage());
    Logger.autoLog(subsystem, logPath + "setOutput",               () -> motor.get());
    Logger.autoLog(subsystem, logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(subsystem, logPath + "appliedVoltage",          () -> motor.getBusVoltage() * motor.getAppliedOutput());
    Logger.autoLog(subsystem, logPath + "powerWatts",              () -> motor.getBusVoltage() * motor.getAppliedOutput() * motor.getOutputCurrent());
    Logger.autoLog(subsystem, logPath + "motorTemperature",        () -> motor.getMotorTemperature());
    Logger.autoLog(subsystem, logPath + "position",                () -> encoder.getPosition());
    Logger.autoLog(subsystem, logPath + "velocity",                () -> encoder.getVelocity());


    StatusChecks.addCheck(subsystem, logPath + "hasFaults", () -> motor.getFaults() == 0);
    // StatusChecks.addCheck(subsystem, logPath + "isConnected", () -> motor.getFirmwareVersion() != 0);
    // StatusChecks.addCheck(subsystem, logPath + "isTooHot", () -> motor.getMotorTemperature() <= NEO.SAFE_TEMPERATURE);
  }

  private static void configure(Supplier<REVLibError> config, CANSparkMax motor) {
    for (int i = 0; i < 5; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
    DriverStation.reportError("Failure configuring spark max " + motor.getDeviceId() + "\n" + "Error: " + config.get().toString(), false);
  }

  public static void configureAndLog550(SubsystemBase subsystem, CANSparkMax motor, boolean inverted, CANSparkMax.IdleMode idleMode) {
    configureAndLog(subsystem, motor, inverted, idleMode, 10, 40);
  }

  public static void configureCANStatusFrames(CANSparkMax motor, int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4, int CANStatus5, int CANStatus6) {
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, CANStatus5);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, CANStatus6);
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

  public static void configurePID(SubsystemBase subsystem, CANSparkMax motor, double kP, double kI, double kD, double kV, boolean wrap) {
    SparkPIDController pid = motor.getPIDController();
    configure(() -> pid.setP(kP, 0), motor);
    configure(() -> pid.setI(kI, 0), motor);
    configure(() -> pid.setD(kD, 0), motor);
    configure(() -> pid.setFF(kV / 12.0, 0), motor);

    if (wrap) {
      configure(() -> pid.setPositionPIDWrappingEnabled(true), motor);
      configure(() -> pid.setPositionPIDWrappingMinInput(-Math.PI), motor);
      configure(() -> pid.setPositionPIDWrappingMaxInput(Math.PI), motor);
    }

    new TunableNumber(subsystem, "PID " + motor.getDeviceId(), pid::setP, 0.0);
  }

  public static void configureEncoder(CANSparkMax motor, double encoderConversionFactor) {
    RelativeEncoder encoder = motor.getEncoder();
    configure(() -> encoder.setPositionConversionFactor(encoderConversionFactor), motor);
    configure(() -> encoder.setVelocityConversionFactor(encoderConversionFactor / 60.0), motor);
  }

  public static void save(CANSparkMax motor) {
    configure(() -> motor.burnFlash(), motor);
  }
}
