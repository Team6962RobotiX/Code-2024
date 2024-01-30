package frc.robot.util.Logging;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveModuleSim;

public final class Logger {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("Logs");
  private static Map<String, Supplier<Object>> entries = new HashMap<String, Supplier<Object>>();

  public static void start() {
    LoggingUpdateCommand command = new LoggingUpdateCommand();

    command.schedule();
  }

  private static void update() {
    for (String key : entries.keySet()) {
      Supplier<Object> supplier = entries.get(key);
      Object object = supplier.get();

      try {
        logObject(key, object);
      } catch (IllegalArgumentException exception) {
        System.out.println("[LOGGING] unknown type: " + object.getClass().getSimpleName());
        System.out.println(exception);
      }
    }

    logRio("roboRio");
  }

  private static class LoggingUpdateCommand extends Command {
    @Override
    public void execute() {
      Logger.update();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
  }

  public static void log(String key, Supplier<Object> supplier) {
    entries.put(key, supplier);
  }

  public static void log(String key, Object obj) {
    log(key, () -> obj);
  }

  public static void logObject(String key, Object obj) {
    if (obj instanceof CANSparkMax) logObject(key, (CANSparkMax) obj);
    else if (obj instanceof RelativeEncoder) logObject(key, (RelativeEncoder) obj);
    else if (obj instanceof AHRS) logObject(key, (AHRS) obj);
    else if (obj instanceof Pose2d) logObject(key, (Pose2d) obj);
    else if (obj instanceof SwerveModuleState) logObject(key, (SwerveModuleState) obj);
    else if (obj instanceof SwerveModuleState[]) logObject(key, (SwerveModuleState[]) obj);
    else if (obj instanceof SwerveModulePosition[]) logObject(key, (SwerveModulePosition[]) obj);
    else if (obj instanceof CANStatus) logObject(key, (CANStatus) obj);
    else if (obj instanceof PowerDistribution) logObject(key, (PowerDistribution) obj);
    else table.getEntry(key).setValue(obj);
  }

  private static void logObject(String path, CANSparkMax sparkMax) {
    logObject(path + "/power", sparkMax.get());
    logObject(path + "/motorTemperature", sparkMax.getMotorTemperature());
    logObject(path + "/outputCurrent", sparkMax.getOutputCurrent());
    logObject(path + "/busVoltage", sparkMax.getBusVoltage());
    logObject(path + "/appliedOutput", sparkMax.getAppliedOutput());
    logObject(path + "/encoder", sparkMax.getEncoder());
  }

  private static void logObject(String path, RelativeEncoder encoder) {
    logObject(path + "/position", encoder.getPosition());
    logObject(path + "/velocity", encoder.getVelocity());
  }

  private static void logObject(String path, CANcoder encoder) {
    logObject(path + "/absolutePosition", encoder.getAbsolutePosition());
    logObject(path + "/position", encoder.getPosition());
    logObject(path + "/velocity", encoder.getVelocity());
  }

  private static void logObject(String path, AHRS navX) {
    logObject(path + "/isAltitudeValid", navX.isAltitudeValid());
    logObject(path + "/isCalibrating", navX.isCalibrating());
    logObject(path + "/isConnected", navX.isConnected());
    logObject(path + "/isMagneticDisturbance", navX.isMagneticDisturbance());
    logObject(path + "/isMagnetometerCalibrated", navX.isMagnetometerCalibrated());
    logObject(path + "/isMoving", navX.isMoving());
    logObject(path + "/isRotating", navX.isRotating());
    logObject(path + "/actualUpdateRate", navX.getActualUpdateRate());
    logObject(path + "/firmwareVersion", navX.getFirmwareVersion());
    logObject(path + "/altitude", navX.getAltitude());
    logObject(path + "/angle", navX.getAngle());
    logObject(path + "/angleAdjustment", navX.getAngleAdjustment());
    logObject(path + "/compassHeading", navX.getCompassHeading());
    logObject(path + "/displacementX", navX.getDisplacementX());
    logObject(path + "/displacementY", navX.getDisplacementY());
    logObject(path + "/displacementZ", navX.getDisplacementZ());
    logObject(path + "/fusedHeading", navX.getFusedHeading());
    logObject(path + "/pitch", navX.getPitch());
    logObject(path + "/pressure", navX.getPressure());
    logObject(path + "/roll", navX.getRoll());
    logObject(path + "/yaw", navX.getYaw());
    logObject(path + "/temperature", navX.getTempC());
    logObject(path + "/velocityX", navX.getVelocityX());
    logObject(path + "/velocityY", navX.getVelocityY());
    logObject(path + "/velocityZ", navX.getVelocityZ());
    logObject(path + "/accelerationX", navX.getRawAccelX());
    logObject(path + "/accelerationY", navX.getRawAccelY());
    logObject(path + "/accelerationZ", navX.getRawAccelZ());
  }

  private static void logObject(String path, Pose2d pose) {
    logObject(path + "_radians", new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
    logObject(path + "_degrees", new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
  }


  private static void logObject(String path, SwerveModuleState swerveModuleState) {
    logObject(path + "/state", new double[] {
      swerveModuleState.angle.getRadians(), 
      swerveModuleState.speedMetersPerSecond
    });
  }

  private static void logObject(String path, SwerveModuleState[] swerveModuleStates) {
    logObject(path + "/states", new double[] {
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

  private static void logObject(String path, SwerveModulePosition[] swerveModulePositions) {
    logObject(path + "/positions", new double[] {
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

  private static void logRio(String path) {
    logObject(path + "/isBrownedOut", RobotController.isBrownedOut());
    logObject(path + "/isSysActive", RobotController.isSysActive());
    logObject(path + "/brownoutVoltage", RobotController.getBrownoutVoltage());
    logObject(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    logObject(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    logObject(path + "/inputCurrent", RobotController.getInputCurrent());
    logObject(path + "/inputVoltage", RobotController.getInputVoltage());
    logObject(path + "/3V3Line/current", RobotController.getCurrent3V3());
    logObject(path + "/5VLine/current", RobotController.getCurrent5V());
    logObject(path + "/6VLine/current", RobotController.getCurrent6V());
    logObject(path + "/3V3Line/enabled", RobotController.getEnabled3V3());
    logObject(path + "/5VLine/enabled", RobotController.getEnabled5V());
    logObject(path + "/6VLine/enabled", RobotController.getEnabled6V());
    logObject(path + "/3V3Line/faultCount", RobotController.getFaultCount3V3());
    logObject(path + "/5VLine/faultCount", RobotController.getFaultCount5V());
    logObject(path + "/6VLine/faultCount", RobotController.getFaultCount6V());
    logObject(path + "/3V3Line/voltage", RobotController.getVoltage3V3());
    logObject(path + "/5VLine/voltage", RobotController.getVoltage5V());
    logObject(path + "/6VLine/voltage", RobotController.getVoltage6V());
    logObject(path + "/canStatus", RobotController.getCANStatus());
  }

  private static void logObject(String path, CANStatus canStatus) {
    logObject(path + "/busOffCount", canStatus.busOffCount);
    logObject(path + "/percentBusUtilization", canStatus.percentBusUtilization);
    logObject(path + "/receiveErrorCount", canStatus.receiveErrorCount);
    logObject(path + "/transmitErrorCount", canStatus.transmitErrorCount);
    logObject(path + "/txFullCount", canStatus.txFullCount);
  }

  private static void logObject(String path, PowerDistribution PDH) {
    logObject(path + "/faults", PDH.getFaults());
    logObject(path + "/canId", PDH.getModule());
    for (int i = 0; i <= 23; i++) {
      logObject(path + "/channels/channel" + i + "Current", PDH.getCurrent(i));
    }
    logObject(path + "/isSwitchableChannelOn", PDH.getSwitchableChannel());
    logObject(path + "/temperature", PDH.getTemperature());
    logObject(path + "/totalCurrent", PDH.getTotalCurrent());
    logObject(path + "/totalJoules", PDH.getTotalEnergy());
    logObject(path + "/totalWatts", PDH.getTotalPower());
    logObject(path + "/voltage", PDH.getVoltage());
  }

  private static void logObject(String path, PowerDistributionFaults faults) {
    logObject(path + "/brownout", faults.Brownout);
    logObject(path + "/canWarning", faults.CanWarning);

    for (int i = 0; i < 24; i++) {
      logObject(path + "/channel" + i + "BreakerFault", faults.getBreakerFault(i));
    }
    
    logObject(path + "/hardwareFault", faults.HardwareFault);
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
