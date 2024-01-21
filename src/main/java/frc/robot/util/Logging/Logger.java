package frc.robot.util.Logging;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.LOGGING;

public final class Logger {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("Logs");
  private static Map<String, Supplier<Object>> entries = new HashMap<String, Supplier<Object>>();

  public static void startLog() {
    TimerTask task = new TimerTask() {
      @Override
      public void run() {
        logAll();
      }
    };

    Timer timer = new Timer();
    
    timer.schedule(task, 0, (long) (LOGGING.LOGGING_PERIOD_MS));
  }

  private static void logAll() {
    for (String key : entries.keySet()) {
      Object value = entries.get(key).get();
      try {
        log(key, value);
      } catch (IllegalArgumentException e) {
        System.out.println("[LOGGING] unknown type: " + value.getClass().getSimpleName());
      }
    }
    logRio("roboRio");
  }

  public static void autoLog(String key, Supplier<Object> supplier) {
    entries.put(key, supplier);
  }

  public static void autoLog(String key, Object obj) {
    autoLog(key, () -> obj);
  }

  public static void log(String key, Object obj) {
    if (obj instanceof CANSparkMax) log(key, (CANSparkMax) obj);
    else if (obj instanceof RelativeEncoder) log(key, (RelativeEncoder) obj);
    else if (obj instanceof AHRS) log(key, (AHRS) obj);
    else if (obj instanceof Pose2d) log(key, (Pose2d) obj);
    else if (obj instanceof SwerveModuleState) log(key, (SwerveModuleState) obj);
    else if (obj instanceof SwerveModuleState[]) log(key, (SwerveModuleState[]) obj);
    else if (obj instanceof SwerveModulePosition[]) log(key, (SwerveModulePosition[]) obj);
    else if (obj instanceof CANStatus) log(key, (CANStatus) obj);
    else if (obj instanceof PowerDistribution) log(key, (PowerDistribution) obj);
    else table.getEntry(key).setValue(obj);
  }

  public static void log(String path, CANSparkMax sparkMax) {
    log(path + "/power", sparkMax.get());
    log(path + "/motorTemperature", sparkMax.getMotorTemperature());
    log(path + "/outputCurrent", sparkMax.getOutputCurrent());
    log(path + "/busVoltage", sparkMax.getBusVoltage());
    log(path + "/appliedOutput", sparkMax.getAppliedOutput());
    log(path + "/encoder", sparkMax.getEncoder());
  }

  public static void log(String path, RelativeEncoder encoder) {
    log(path + "/position", encoder.getPosition());
    log(path + "/velocity", encoder.getVelocity());
  }

  public static void log(String path, CANcoder encoder) {
    log(path + "/absolutePosition", encoder.getAbsolutePosition());
    log(path + "/position", encoder.getPosition());
    log(path + "/velocity", encoder.getVelocity());
  }

  public static void log(String path, AHRS navX) {
    log(path + "/isAltitudeValid", navX.isAltitudeValid());
    log(path + "/isCalibrating", navX.isCalibrating());
    log(path + "/isConnected", navX.isConnected());
    log(path + "/isMagneticDisturbance", navX.isMagneticDisturbance());
    log(path + "/isMagnetometerCalibrated", navX.isMagnetometerCalibrated());
    log(path + "/isMoving", navX.isMoving());
    log(path + "/isRotating", navX.isRotating());
    log(path + "/actualUpdateRate", navX.getActualUpdateRate());
    log(path + "/firmwareVersion", navX.getFirmwareVersion());
    log(path + "/altitude", navX.getAltitude());
    log(path + "/angle", navX.getAngle());
    log(path + "/angleAdjustment", navX.getAngleAdjustment());
    log(path + "/compassHeading", navX.getCompassHeading());
    log(path + "/displacementX", navX.getDisplacementX());
    log(path + "/displacementY", navX.getDisplacementY());
    log(path + "/displacementZ", navX.getDisplacementZ());
    log(path + "/fusedHeading", navX.getFusedHeading());
    log(path + "/pitch", navX.getPitch());
    log(path + "/pressure", navX.getPressure());
    log(path + "/roll", navX.getRoll());
    log(path + "/yaw", navX.getYaw());
    log(path + "/temperature", navX.getTempC());
    log(path + "/velocityX", navX.getVelocityX());
    log(path + "/velocityY", navX.getVelocityY());
    log(path + "/velocityZ", navX.getVelocityZ());
    log(path + "/accelerationX", navX.getRawAccelX());
    log(path + "/accelerationY", navX.getRawAccelY());
    log(path + "/accelerationZ", navX.getRawAccelZ());
  }

  public static void log(String path, Pose2d pose) {
    log(path + "_radians", new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
    log(path + "_degrees", new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
  }


  public static void log(String path, SwerveModuleState swerveModuleState) {
    log(path + "/state", new double[] {
      swerveModuleState.angle.getRadians(), 
      swerveModuleState.speedMetersPerSecond
    });
  }

  public static void log(String path, SwerveModuleState[] swerveModuleStates) {
    log(path + "/states", new double[] {
      swerveModuleStates[0].angle.getRadians(), 
      swerveModuleStates[0].speedMetersPerSecond, 
      swerveModuleStates[1].angle.getRadians(), 
      swerveModuleStates[1].speedMetersPerSecond, 
      swerveModuleStates[2].angle.getRadians(), 
      swerveModuleStates[2].speedMetersPerSecond, 
      swerveModuleStates[3].angle.getRadians(), 
      swerveModuleStates[3].speedMetersPerSecond, 
    });
  }

  public static void log(String path, SwerveModulePosition[] swerveModulePositions) {
    log(path + "/positions", new double[] {
      swerveModulePositions[0].angle.getRadians(), 
      swerveModulePositions[0].distanceMeters, 
      swerveModulePositions[1].angle.getRadians(), 
      swerveModulePositions[1].distanceMeters, 
      swerveModulePositions[2].angle.getRadians(), 
      swerveModulePositions[2].distanceMeters, 
      swerveModulePositions[3].angle.getRadians(), 
      swerveModulePositions[3].distanceMeters, 
    });
  }

  public static void logRio(String path) {
    log(path + "/isBrownedOut", RobotController.isBrownedOut());
    log(path + "/isSysActive", RobotController.isSysActive());
    log(path + "/brownoutVoltage", RobotController.getBrownoutVoltage());
    log(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    log(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    log(path + "/inputCurrent", RobotController.getInputCurrent());
    log(path + "/inputVoltage", RobotController.getInputVoltage());
    log(path + "/3V3Line/current", RobotController.getCurrent3V3());
    log(path + "/5VLine/current", RobotController.getCurrent5V());
    log(path + "/6VLine/current", RobotController.getCurrent6V());
    log(path + "/3V3Line/enabled", RobotController.getEnabled3V3());
    log(path + "/5VLine/enabled", RobotController.getEnabled5V());
    log(path + "/6VLine/enabled", RobotController.getEnabled6V());
    log(path + "/3V3Line/faultCount", RobotController.getFaultCount3V3());
    log(path + "/5VLine/faultCount", RobotController.getFaultCount5V());
    log(path + "/6VLine/faultCount", RobotController.getFaultCount6V());
    log(path + "/3V3Line/voltage", RobotController.getVoltage3V3());
    log(path + "/5VLine/voltage", RobotController.getVoltage5V());
    log(path + "/6VLine/voltage", RobotController.getVoltage6V());
    log(path + "/canStatus", RobotController.getCANStatus());
  }

  public static void log(String path, CANStatus canStatus) {
    log(path + "/busOffCount", canStatus.busOffCount);
    log(path + "/percentBusUtilization", canStatus.percentBusUtilization);
    log(path + "/receiveErrorCount", canStatus.receiveErrorCount);
    log(path + "/transmitErrorCount", canStatus.transmitErrorCount);
    log(path + "/txFullCount", canStatus.txFullCount);
  }

  public static void log(String path, PowerDistribution PDH) {
    log(path + "/faults", PDH.getFaults());
    log(path + "/canId", PDH.getModule());
    for (int i = 0; i <= 23; i++) {
      log(path + "/channels/channel" + i + "Current", PDH.getCurrent(i));
    }
    log(path + "/isSwitchableChannelOn", PDH.getSwitchableChannel());
    log(path + "/temperature", PDH.getTemperature());
    log(path + "/totalCurrent", PDH.getTotalCurrent());
    log(path + "/totalJoules", PDH.getTotalEnergy());
    log(path + "/totalWatts", PDH.getTotalPower());
    log(path + "/voltage", PDH.getVoltage());
  }

  public static void log(String path, PowerDistributionFaults faults) {
    log(path + "/brownout", faults.Brownout);
    log(path + "/canWarning", faults.CanWarning);

    for (int i = 0; i < 24; i++) {
      log(path + "/channel" + i + "BreakerFault", faults.getBreakerFault(i));
    }
    
    log(path + "/hardwareFault", faults.HardwareFault);
  }

  public static void log(String path, Object self, Class clazz) {    
    for (Class c: clazz.getDeclaredClasses()) {
      try {
        log(path + "/" + c.getSimpleName(), self, c);
      }
      catch (Exception e) {}
    }

    for (Field f: clazz.getDeclaredFields()) {
      try {
        log(path + "/" + f.getName(), f.get(self));
      }
      catch (Exception e) {}
    }
  }
}
