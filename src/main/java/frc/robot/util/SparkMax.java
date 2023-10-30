package frc.robot.util;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.NEO;
import frc.robot.util.Logging.Logger;

public class SparkMax extends CANSparkMax {
  private String name = "";
  private String logPath;

  public SparkMax(int deviceId, MotorType type) {
    super(deviceId, type);
    initialize();
  }

  public SparkMax(int deviceId, MotorType type, String name) {
    super(deviceId, type);
    this.name = name;
    initialize();
  }

  private void initialize() {
    restoreFactoryDefaults();
    setIdleMode(IdleMode.kBrake);
    setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, 80);
    if (name.equals("")) {
      logPath = "motorControllers/SparkMAX " + getDeviceId();
    } else {
      logPath = "motorControllers/SparkMAX " + getDeviceId() + " (" + name + ")";
    }
    Logger.autoLog(logPath, () -> this);
  }

  public String getLogPath() {
    return logPath;
  }

  public void set(double speed) {
    super.set(speed);
    Logger.log(logPath + "/calls/set", speed);
  }

  public void setVoltage(double outputVolts) {
    super.set(outputVolts);
    Logger.log(logPath + "/calls/setVoltage", outputVolts);
  }

  public double get() {
    double get = super.get();
    Logger.log(logPath + "/calls/get", get);
    return get;
  }

  public void setInverted(boolean isInverted) {
    super.setInverted(isInverted);
    Logger.log(logPath + "/calls/setInverted", isInverted);
  }

  public boolean getInverted() {
    boolean getInverted = super.getInverted();
    Logger.log(logPath + "/calls/getInverted", getInverted);
    return getInverted;
  }

  public void disable() {
    super.disable();
    Logger.log(logPath + "/calls/disable", "disabling");
  }

  public void stopMotor() {
    super.stopMotor();
    Logger.log(logPath + "/calls/stopMotor", "stopping");
  }

  public SparkMaxEncoder getRelativeEncoder() {
    return new SparkMaxEncoder(super.getEncoder(), this);
  }

  @Deprecated
  public RelativeEncoder getEncoder() {
    return new SparkMaxEncoder(super.getEncoder(), this);
  }
  
  @Deprecated
  public RelativeEncoder getEncoder(SparkMaxRelativeEncoder.Type encoderType, int countsPerRev) {
    return new SparkMaxEncoder(super.getEncoder(encoderType, countsPerRev), this);
  }

  @Deprecated
  public RelativeEncoder getEncoder(com.revrobotics.EncoderType encoderType, int countsPerRev) {
    return new SparkMaxEncoder(super.getEncoder(encoderType, countsPerRev), this);
  }

  public RelativeEncoder getAlternateEncoder(int countsPerRev) {
    return new SparkMaxEncoder(super.getAlternateEncoder(countsPerRev), this);
  }

  public RelativeEncoder getAlternateEncoder(SparkMaxAlternateEncoder.Type encoderType, int countsPerRev) {
    return new SparkMaxEncoder(super.getAlternateEncoder(encoderType, countsPerRev), this);
  }

  public SparkMaxPID getOnboardController() {
    return new SparkMaxPID(this);
  }

  @Deprecated
  public SparkMaxPIDController getPIDController() {
    return super.getPIDController();
  }

  public REVLibError setSmartCurrentLimit(int limit) {
    return execute(() -> super.setSmartCurrentLimit(limit), "setSmartCurrentLimit");
  }

  public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit) {
    return execute(() -> super.setSmartCurrentLimit(stallLimit, freeLimit), "setSmartCurrentLimit");
  }

  public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
    return execute(() -> super.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM), "setSmartCurrentLimit");
  }

  public REVLibError setSecondaryCurrentLimit(double limit) {
    return execute(() -> super.setSecondaryCurrentLimit(limit), "setSecondaryCurrentLimit");
  }

  public REVLibError setSecondaryCurrentLimit(double limit, int chopCycles) {
    return execute(() -> super.setSecondaryCurrentLimit(limit, chopCycles), "setSecondaryCurrentLimit");
  }

  public REVLibError setIdleMode(IdleMode mode) {
    return execute(() -> super.setIdleMode(mode), "setIdleMode");
  }

  public REVLibError enableVoltageCompensation(double nominalVoltage) {
    return execute(() -> super.enableVoltageCompensation(nominalVoltage), "enableVoltageCompensation");
  }

  public REVLibError disableVoltageCompensation() {
    return execute(() -> super.disableVoltageCompensation(), "disableVoltageCompensation");
  }

  @Deprecated
  public REVLibError setOpenLoopRampRate(double rate) {
    return execute(() -> super.setOpenLoopRampRate(rate), "setOpenLoopRampRate");
  }

  @Deprecated
  public REVLibError setClosedLoopRampRate(double rate) {
    return execute(() -> super.setClosedLoopRampRate(rate), "setClosedLoopRampRate");
  }

  public REVLibError follow(final CANSparkMax leader) {
    return execute(() -> super.follow(leader), "follow");
  }

  public REVLibError follow(final CANSparkMax leader, boolean invert) {
    return execute(() -> super.follow(leader, invert), "follow");
  }

  public REVLibError follow(ExternalFollower leader, int deviceID) {
    return execute(() -> super.follow(leader, deviceID), "follow");
  }

  public REVLibError follow(ExternalFollower leader, int deviceID, boolean invert) {
    return execute(() -> super.follow(leader, deviceID, invert), "follow");
  }

  public REVLibError clearFaults() {
    return execute(() -> super.clearFaults(), "clearFaults");
  }

  public REVLibError burnFlash() {
    return execute(() -> super.burnFlash(), "burnFlash");
  }

  public REVLibError setCANTimeout(int milliseconds) {
    return execute(() -> super.setCANTimeout(milliseconds), "setCANTimeout");
  }

  public REVLibError enableSoftLimit(SoftLimitDirection direction, boolean enable) {
    return execute(() -> super.enableSoftLimit(direction, enable), "enableSoftLimit");
  }

  public REVLibError setSoftLimit(SoftLimitDirection direction, float limit) {
    return execute(() -> super.setSoftLimit(direction, limit), "setSoftLimit");
  }

  public REVLibError getLastError() {
    return super.getLastError();
  }

  public void setPeriodicFramePeriods(int[] statusFramePeriods) {
    setPeriodicFramePeriod(PeriodicFrame.kStatus0, statusFramePeriods[0]);
    setPeriodicFramePeriod(PeriodicFrame.kStatus1, statusFramePeriods[1]);
    setPeriodicFramePeriod(PeriodicFrame.kStatus2, statusFramePeriods[2]);
    setPeriodicFramePeriod(PeriodicFrame.kStatus3, statusFramePeriods[3]);
    setPeriodicFramePeriod(PeriodicFrame.kStatus4, statusFramePeriods[4]);
    setPeriodicFramePeriod(PeriodicFrame.kStatus5, statusFramePeriods[5]);
    setPeriodicFramePeriod(PeriodicFrame.kStatus6, statusFramePeriods[6]);
  }

  public void setRampRate(double rampRate) {
    setOpenLoopRampRate(rampRate);
    setClosedLoopRampRate(rampRate);
  }

  public REVLibError execute(Supplier<REVLibError> supplier, String action) {
    REVLibError status = supplier.get();
    if (status == REVLibError.kOk) {
      Logger.log(logPath + "/calls/configuration/" + action, "success");
      return status;
    }
    DriverStation.reportError(String.format("SparkMAX %s (CAN %s) FAILED ACTION: %s, REASON: %s", name, getDeviceId(), action, status.toString()), false);
    new java.util.Timer().schedule(new java.util.TimerTask() {
      @Override
      public void run() {
        execute(supplier, action);
      }
    }, 1000);
    return status;
  }
}
