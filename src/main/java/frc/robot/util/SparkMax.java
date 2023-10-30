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
  private RelativeEncoder encoder;
  private SparkMaxPIDController controller;

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
    encoder = getEncoder();
    controller = getPIDController();
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
