package frc.robot.utils;

import java.lang.reflect.Field;
import java.sql.Driver;
import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public final class Logger {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("Logs");

  public static void logDriverStation(String path) {
    DriverStation.startDataLog(DataLogManager.getLog(), true);
  }

  public static void logSparkMax(String path, CANSparkMax sparkMax) {
    logValue(path + "/power", sparkMax.get());
    logValue(path + "/appliedOutputDutyCycle", sparkMax.getAppliedOutput());
    logValue(path + "/busVoltage", sparkMax.getBusVoltage());
    logValue(path + "/motorTemperature", sparkMax.getMotorTemperature());
    logValue(path + "/outputCurrent", sparkMax.getOutputCurrent());
    logValue(path + "/faults", sparkMax.getFaults());
    logValue(path + "/firmwareString", sparkMax.getFirmwareString());
    logValue(path + "/firmwareVersion", sparkMax.getFirmwareVersion());
    logValue(path + "/stickyFaults", sparkMax.getStickyFaults());
    logValue(path + "/isFollower", sparkMax.isFollower());
    logRelativeEncoder(path + "/relativeEncoder", sparkMax.getEncoder());
    checkSparkMaxStatus(sparkMax);
  }

  public static void logRelativeEncoder(String path, RelativeEncoder encoder) {
    logValue(path + "/position", encoder.getPosition());
    logValue(path + "/velocity", encoder.getVelocity());
    logValue(path + "/positionConversionFactor", encoder.getPositionConversionFactor());
    logValue(path + "/velocityConversionFactor", encoder.getVelocityConversionFactor());
    logValue(path + "/nativePosition", encoder.getPosition() / encoder.getPositionConversionFactor());
    logValue(path + "/nativeVelocity", encoder.getVelocity() / encoder.getVelocityConversionFactor());
  }

  public static void logCANCoder(String path, CANCoder encoder) {
    logValue(path + "/absolutePosition", encoder.getAbsolutePosition());
    logValue(path + "/voltage", encoder.getBusVoltage());
    logValue(path + "/firmwareVersion", encoder.getFirmwareVersion());
    logValue(path + "/magnetFieldStrength", encoder.getMagnetFieldStrength().value);
    logValue(path + "/position", encoder.getPosition());
    logValue(path + "/velocity", encoder.getVelocity());

    CANCoderFaults faults = new CANCoderFaults();
    encoder.getFaults(new CANCoderFaults());

    logCANCoderFaults(path + "/faults", faults);
  }

  public static void logCANCoderFaults(String path, CANCoderFaults faults) {
    logValue(path + "/hardwareFault", faults.HardwareFault);
    logValue(path + "/APIError", faults.APIError);
    logValue(path + "/magnetTooWeak", faults.MagnetTooWeak);
    logValue(path + "/resetDuringEn", faults.ResetDuringEn);
    logValue(path + "/underVoltage", faults.UnderVoltage);
  }

  public static void logNavX(String path, AHRS navX) {
    logValue(path + "/isAltitudeValid", navX.isAltitudeValid());
    logValue(path + "/isCalibrating", navX.isCalibrating());
    logValue(path + "/isConnected", navX.isConnected());
    logValue(path + "/isMagneticDisturbance", navX.isMagneticDisturbance());
    logValue(path + "/isMagnetometerCalibrated", navX.isMagnetometerCalibrated());
    logValue(path + "/isMoving", navX.isMoving());
    logValue(path + "/isRotating", navX.isRotating());
    logValue(path + "/actualUpdateRate", navX.getActualUpdateRate());
    logValue(path + "/firmwareVersion", navX.getFirmwareVersion());
    logValue(path + "/altitude", navX.getAltitude());
    logValue(path + "/angle", navX.getAngle());
    logValue(path + "/angleAdjustment", navX.getAngleAdjustment());
    logValue(path + "/compassHeading", navX.getCompassHeading());
    logValue(path + "/displacementX", navX.getDisplacementX());
    logValue(path + "/displacementY", navX.getDisplacementY());
    logValue(path + "/displacementZ", navX.getDisplacementZ());
    logValue(path + "/fusedHeading", navX.getFusedHeading());
    logValue(path + "/pitch", navX.getPitch());
    logValue(path + "/pressure", navX.getPressure());
    logValue(path + "/roll", navX.getRoll());
    logValue(path + "/yaw", navX.getYaw());
    logValue(path + "/temperature", navX.getTempC());
    logValue(path + "/velocityX", navX.getVelocityX());
    logValue(path + "/velocityY", navX.getVelocityY());
    logValue(path + "/velocityZ", navX.getVelocityZ());
    logValue(path + "/accelerationX", navX.getRawAccelX());
    logValue(path + "/accelerationY", navX.getRawAccelY());
    logValue(path + "/accelerationZ", navX.getRawAccelZ());
  }

  public static void logPose(String path, Pose2d pose) {
    logValue(path, new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
  }

  public static void logModuleStates(String path, SwerveModuleState[] targetModuleStates, SwerveModuleState[] measuredModuleStates, SwerveModulePosition[] modulePositions) {
        logValue(path + "/modulePositions", new double[] {
      modulePositions[0].angle.getRadians(), 
      modulePositions[0].distanceMeters, 
      modulePositions[1].angle.getRadians(), 
      modulePositions[1].distanceMeters, 
      modulePositions[2].angle.getRadians(), 
      modulePositions[2].distanceMeters, 
      modulePositions[3].angle.getRadians(), 
      modulePositions[3].distanceMeters, 
    });

    logValue(path + "/targetModuleStates", new double[] { 
      targetModuleStates[0].angle.getRadians(), 
      targetModuleStates[0].speedMetersPerSecond, 
      targetModuleStates[1].angle.getRadians(), 
      targetModuleStates[1].speedMetersPerSecond, 
      targetModuleStates[2].angle.getRadians(), 
      targetModuleStates[2].speedMetersPerSecond, 
      targetModuleStates[3].angle.getRadians(), 
      targetModuleStates[3].speedMetersPerSecond, 
    });

    logValue(path + "/measuredModuleStates", new double[] { 
      measuredModuleStates[0].angle.getRadians(), 
      measuredModuleStates[0].speedMetersPerSecond, 
      measuredModuleStates[1].angle.getRadians(), 
      measuredModuleStates[1].speedMetersPerSecond, 
      measuredModuleStates[2].angle.getRadians(), 
      measuredModuleStates[2].speedMetersPerSecond, 
      measuredModuleStates[3].angle.getRadians(), 
      measuredModuleStates[3].speedMetersPerSecond, 
    });
  }

  public static void logRobotController(String path) {
    logValue(path + "/isBrownedOut", RobotController.isBrownedOut());
    logValue(path + "/isSysActive", RobotController.isSysActive());
    logValue(path + "/brownoutVoltage", RobotController.getBrownoutVoltage());
    logValue(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    logValue(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    logValue(path + "/inputCurrent", RobotController.getInputCurrent());
    logValue(path + "/inputVoltage", RobotController.getInputVoltage());
    logValue(path + "/3V3Line/current", RobotController.getCurrent3V3());
    logValue(path + "/5VLine/current", RobotController.getCurrent5V());
    logValue(path + "/6VLine/current", RobotController.getCurrent6V());
    logValue(path + "/3V3Line/enabled", RobotController.getEnabled3V3());
    logValue(path + "/5VLine/enabled", RobotController.getEnabled5V());
    logValue(path + "/6VLine/enabled", RobotController.getEnabled6V());
    logValue(path + "/3V3Line/faultCount", RobotController.getFaultCount3V3());
    logValue(path + "/5VLine/faultCount", RobotController.getFaultCount5V());
    logValue(path + "/6VLine/faultCount", RobotController.getFaultCount6V());
    logValue(path + "/3V3Line/voltage", RobotController.getVoltage3V3());
    logValue(path + "/5VLine/voltage", RobotController.getVoltage5V());
    logValue(path + "/6VLine/voltage", RobotController.getVoltage6V());
    logCanStatus(path + "/canStatus", RobotController.getCANStatus());
  }

  public static void logCanStatus(String path, CANStatus canStatus) {
    logValue(path + "/busOffCount", canStatus.busOffCount);
    logValue(path + "/percentBusUtilization", canStatus.percentBusUtilization);
    logValue(path + "/receiveErrorCount", canStatus.receiveErrorCount);
    logValue(path + "/transmitErrorCount", canStatus.transmitErrorCount);
    logValue(path + "/txFullCount", canStatus.txFullCount);
  }

  public static void logPDH(String path, PowerDistribution PDH) {
    logPDHFaults(path + "/faults", PDH.getFaults());

    logValue(path + "/canId", PDH.getModule());
    for (int i = 0; i <= 23; i++) {
      logValue(path + "/channels/channel" + i + "Current", PDH.getCurrent(i));
    }
    logValue(path + "/isSwitchableChannelOn", PDH.getSwitchableChannel());
    logValue(path + "/temperature", PDH.getTemperature());
    logValue(path + "/totalCurrent", PDH.getTotalCurrent());
    logValue(path + "/totalJoules", PDH.getTotalEnergy());
    logValue(path + "/totalWatts", PDH.getTotalPower());
    logValue(path + "/voltage", PDH.getVoltage());
  }

  public static void logPDHFaults(String path, PowerDistributionFaults faults) {
    logValue(path + "/brownout", faults.Brownout);
    logValue(path + "/canWarning", faults.CanWarning);
    logValue(path + "/channel0BreakerFault", faults.Channel0BreakerFault);
    logValue(path + "/channel1BreakerFault", faults.Channel1BreakerFault);
    logValue(path + "/channel2BreakerFault", faults.Channel2BreakerFault);
    logValue(path + "/channel3BreakerFault", faults.Channel3BreakerFault);
    logValue(path + "/channel4BreakerFault", faults.Channel4BreakerFault);
    logValue(path + "/channel5BreakerFault", faults.Channel5BreakerFault);
    logValue(path + "/channel6BreakerFault", faults.Channel6BreakerFault);
    logValue(path + "/channel7BreakerFault", faults.Channel7BreakerFault);
    logValue(path + "/channel8BreakerFault", faults.Channel8BreakerFault);
    logValue(path + "/channel9BreakerFault", faults.Channel9BreakerFault);
    logValue(path + "/channel10BreakerFault", faults.Channel10BreakerFault);
    logValue(path + "/channel11BreakerFault", faults.Channel11BreakerFault);
    logValue(path + "/channel12BreakerFault", faults.Channel12BreakerFault);
    logValue(path + "/channel13BreakerFault", faults.Channel13BreakerFault);
    logValue(path + "/channel14BreakerFault", faults.Channel14BreakerFault);
    logValue(path + "/channel15BreakerFault", faults.Channel15BreakerFault);
    logValue(path + "/channel16BreakerFault", faults.Channel16BreakerFault);
    logValue(path + "/channel17BreakerFault", faults.Channel17BreakerFault);
    logValue(path + "/channel18BreakerFault", faults.Channel18BreakerFault);
    logValue(path + "/channel19BreakerFault", faults.Channel19BreakerFault);
    logValue(path + "/channel20BreakerFault", faults.Channel20BreakerFault);
    logValue(path + "/channel21BreakerFault", faults.Channel21BreakerFault);
    logValue(path + "/channel22BreakerFault", faults.Channel22BreakerFault);
    logValue(path + "/channel23BreakerFault", faults.Channel23BreakerFault);
    logValue(path + "/hardwareFault", faults.HardwareFault);
  }

  public static void logValue(String key, Object value) {
    if (value instanceof Pose2d) logPose(key, (Pose2d) value);
    NetworkTableEntry entry = getEntry(key);
    entry.setValue(value);
  }

  public static NetworkTableEntry getEntry(String key) {
    return table.getEntry(key);
  }

  public static void REV(REVLibError error, String message) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(String.format("[FAILED] %s: %s", message, error.toString()), false);
    }
  }

  public static void checkSparkMaxStatus(CANSparkMax sparkMax){
    Set<String> faults = new HashSet<>();
    if (sparkMax.getFaults() != 0) {
      for (CANSparkMax.FaultID fault : CANSparkMax.FaultID.values()){
        if (sparkMax.getFault(fault)) {
          faults.add(fault.toString());
        }
      }
      DriverStation.reportError("[FAULTS] Spark Max " + sparkMax.getDeviceId() + " has faults: " + faults, false);
    }
    if (sparkMax.getMotorTemperature() > Constants.NEO.SAFE_TEMPERATURE) {
      DriverStation.reportError("[TEMPERATURE] Spark Max " + sparkMax.getDeviceId() + " has been disabled for being too hot. (" + sparkMax.getMotorTemperature() + ")", false);
      sparkMax.stopMotor();
    }
  }

  public static void logClassValues(String path, Object self, Class clazz) {
    for (Class c: clazz.getDeclaredClasses()) {
      try {
        logClassValues(path + "/" + c.getSimpleName(), self, c);
      }
      catch (Exception e) {}
    }

    for (Field f: clazz.getDeclaredFields()) {
      try {
        logValue(path + "/" + f.getName(), f.get(self));
      }
      catch (Exception e) {}
    }
  }
}
