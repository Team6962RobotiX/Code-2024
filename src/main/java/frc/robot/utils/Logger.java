package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Logging;
import frc.robot.subsystems.SwerveDrive;

public class Logger {
  DataLog log = DataLogManager.getLog();
  Map<String, Object> logEntries = new HashMap<String, Object>();
  SwerveDrive drive;
  PowerDistribution PDH = new PowerDistribution(CAN.PDH, ModuleType.kRev);

  public NetworkTableInstance instance = NetworkTableInstance.getDefault();

  public Logger(SwerveDrive drive) {
    this.drive = drive;
  }

  public void logAll() {
    if (Logging.ENABLE_DRIVE)
      logSwerve("/swerveDrive", drive);
    if (Logging.ENABLE_PDH)
      logPDH("/powerDistribution", PDH);
    if (Logging.ENABLE_ROBOT_CONTROLLER)
      logRobotController("/robotController");
    if (Logging.ENABLE_DRIVER_STATION)
      logDriverStation("/driverStation");
  }

  public void logDriverStation(String path) {
    DriverStation.startDataLog(log, true);
  }

  public void logSwerve(String path, SwerveDrive drive) {
    logNavX(path + "/gyro", drive.getGyro());
    logOdometer(path + "/odometer", drive.getOdometer());
    logData(path + "/heading", drive.getRotation2d().getRadians());
    logData(path + "/totalCurrent", drive.getCurrent());
    logPose(path + "/pose", drive.getPose());
    logModuleStates(path + "/moduleStates", drive.getTargetModuleStates(), drive.getMeasuredModuleStates(), drive.getModulePositions());

    for (SwerveModule module : drive.getModules()) {
      logSwerveModule(path + "/modules/" + module.getName(), module);
    }
  }

  public void logSwerveModule(String path, SwerveModule module) {
    logSparkMax(path + "/driveMotor", module.getMotors()[0]);
    logSparkMax(path + "/steerMotor", module.getMotors()[1]);
    logCANCoder(path + "/canCoder", module.getCanCoder());
    logData(path + "/steerDegrees", module.getSteerDirection().getDegrees());
    logData(path + "/velocity", module.getVelocity());
  }

  public void logSparkMax(String path, CANSparkMax sparkMax) {
    logData(path + "/power", sparkMax.get());
    logData(path + "/appliedOutputDutyCycle", sparkMax.getAppliedOutput());
    logData(path + "/busVoltage", sparkMax.getBusVoltage());
    logData(path + "/motorTemperature", sparkMax.getMotorTemperature());
    logData(path + "/outputCurrent", sparkMax.getOutputCurrent());
    logData(path + "/faults", sparkMax.getFaults());
    logData(path + "/firmwareString", sparkMax.getFirmwareString(), true);
    logData(path + "/firmwareVersion", sparkMax.getFirmwareVersion(), true);
    logData(path + "/stickyFaults", sparkMax.getStickyFaults(), true);
    logRelativeEncoder(path + "/relativeEncoder", sparkMax.getEncoder());
  }

  public void logRelativeEncoder(String path, RelativeEncoder encoder) {
    logData(path + "/position", encoder.getPosition());
    logData(path + "/velocity", encoder.getVelocity());
  }

  public void logCANCoder(String path, CANCoder encoder) {
    logData(path + "/absolutePosition", encoder.getAbsolutePosition());
    logData(path + "/voltage", encoder.getBusVoltage());
    logData(path + "/firmwareVersion", encoder.getFirmwareVersion(), true);
    logData(path + "/magnetFieldStrength", encoder.getMagnetFieldStrength().value);
    logData(path + "/position", encoder.getPosition());
    logData(path + "/velocity", encoder.getVelocity());

    CANCoderFaults faults = new CANCoderFaults();
    encoder.getFaults(new CANCoderFaults());

    logCANCoderFaults(path + "/faults", faults);
  }

  public void logCANCoderFaults(String path, CANCoderFaults faults) {
    logData(path + "/hardwareFault", faults.HardwareFault);
    logData(path + "/APIError", faults.APIError);
    logData(path + "/magnetTooWeak", faults.MagnetTooWeak);
    logData(path + "/resetDuringEn", faults.ResetDuringEn);
    logData(path + "/underVoltage", faults.UnderVoltage);
  }

  public void logNavX(String path, AHRS navX) {
    logData(path + "/actualUpdateRate", navX.getActualUpdateRate(), true);
    logData(path + "/firmwareVersion", navX.getFirmwareVersion(), true);
    logData(path + "/altitude", navX.getAltitude());
    logData(path + "/angle", navX.getAngle());
    logData(path + "/angleAdjustment", navX.getAngleAdjustment());
    logData(path + "/compassHeading", navX.getCompassHeading());
    logData(path + "/displacementX", navX.getDisplacementX());
    logData(path + "/displacementY", navX.getDisplacementY());
    logData(path + "/displacementZ", navX.getDisplacementZ());
    logData(path + "/fusedHeading", navX.getFusedHeading());
    logData(path + "/pitch", navX.getPitch());
    logData(path + "/pressure", navX.getPressure());
    logData(path + "/roll", navX.getRoll());
    logData(path + "/yaw", navX.getYaw());
    logData(path + "/temperature", navX.getTempC());
    logData(path + "/velocityX", navX.getVelocityX());
    logData(path + "/velocityY", navX.getVelocityY());
    logData(path + "/velocityZ", navX.getVelocityZ());
    logData(path + "/accelerationX", navX.getRawAccelX());
    logData(path + "/accelerationY", navX.getRawAccelY());
    logData(path + "/accelerationZ", navX.getRawAccelZ());
  }

  public void logOdometer(String path, SwerveDriveOdometry odometer) {
    logData(path + "/odometer", new double[] { odometer.getPoseMeters().getX(), odometer.getPoseMeters().getY(), odometer.getPoseMeters().getRotation().getRadians() });
  }

  public void logPose(String path, Pose2d pose) {
    logData(path, new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
  }

  public void logModuleStates(String path, SwerveModuleState[] targetModuleStates, SwerveModuleState[] measuredModuleStates, SwerveModulePosition[] modulePositions) {
    logData(path + "/modulePositions", new double[] {
        modulePositions[0].angle.getRadians(), modulePositions[0].distanceMeters,
        modulePositions[1].angle.getRadians(), modulePositions[1].distanceMeters,
        modulePositions[2].angle.getRadians(), modulePositions[2].distanceMeters,
        modulePositions[3].angle.getRadians(), modulePositions[3].distanceMeters,
    });

    logData(path + "/targetModuleStates", new double[] {
        targetModuleStates[0].angle.getRadians(), targetModuleStates[0].speedMetersPerSecond,
        targetModuleStates[1].angle.getRadians(), targetModuleStates[1].speedMetersPerSecond,
        targetModuleStates[2].angle.getRadians(), targetModuleStates[2].speedMetersPerSecond,
        targetModuleStates[3].angle.getRadians(), targetModuleStates[3].speedMetersPerSecond,
    });

    logData(path + "/measuredModuleStates", new double[] {
        measuredModuleStates[0].angle.getRadians(), measuredModuleStates[0].speedMetersPerSecond,
        measuredModuleStates[1].angle.getRadians(), measuredModuleStates[1].speedMetersPerSecond,
        measuredModuleStates[2].angle.getRadians(), measuredModuleStates[2].speedMetersPerSecond,
        measuredModuleStates[3].angle.getRadians(), measuredModuleStates[3].speedMetersPerSecond,
    });
  }

  public void logRobotController(String path) {
    logData(path + "/brownoutVoltage", RobotController.getBrownoutVoltage(), true);
    logData(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    logData(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    logData(path + "/inputCurrent", RobotController.getInputCurrent());
    logData(path + "/inputVoltage", RobotController.getInputVoltage());
    logData(path + "/3V3Line/current", RobotController.getCurrent3V3());
    logData(path + "/5VLine/current", RobotController.getCurrent5V());
    logData(path + "/6VLine/current", RobotController.getCurrent6V());
    logData(path + "/3V3Line/enabled", RobotController.getEnabled3V3());
    logData(path + "/5VLine/enabled", RobotController.getEnabled5V());
    logData(path + "/6VLine/enabled", RobotController.getEnabled6V());
    logData(path + "/3V3Line/faultCount", RobotController.getFaultCount3V3());
    logData(path + "/5VLine/faultCount", RobotController.getFaultCount5V());
    logData(path + "/6VLine/faultCount", RobotController.getFaultCount6V());
    logData(path + "/3V3Line/voltage", RobotController.getVoltage3V3());
    logData(path + "/5VLine/voltage", RobotController.getVoltage5V());
    logData(path + "/6VLine/voltage", RobotController.getVoltage6V());
    logCanStatus(path + "/canStatus", RobotController.getCANStatus());
  }

  public void logCanStatus(String path, CANStatus canStatus) {
    logData(path + "/busOffCount", canStatus.busOffCount);
    logData(path + "/percentBusUtilization", canStatus.percentBusUtilization);
    logData(path + "/receiveErrorCount", canStatus.receiveErrorCount);
    logData(path + "/transmitErrorCount", canStatus.transmitErrorCount);
    logData(path + "/txFullCount", canStatus.txFullCount);
  }

  public void logPDH(String path, PowerDistribution PDH) {
    logPDHFaults(path + "/faults", PDH.getFaults());

    logData(path + "/canId", PDH.getModule(), true);
    for (int i = 0; i <= 23; i++) {
      logData(path + "/channels/channel" + i + "Current", PDH.getCurrent(i));
    }
    logData(path + "/isSwitchableChannelOn", PDH.getSwitchableChannel(), true);
    logData(path + "/temperature", PDH.getTemperature());
    logData(path + "/totalCurrent", PDH.getTotalCurrent());
    logData(path + "/totalJoules", PDH.getTotalEnergy());
    logData(path + "/totalWatts", PDH.getTotalPower());
    logData(path + "/voltage", PDH.getVoltage());
  }

  public void logPDHFaults(String path, PowerDistributionFaults faults) {
    logData(path + "/brownout", faults.Brownout);
    logData(path + "/canWarning", faults.CanWarning);
    logData(path + "/channel0BreakerFault", faults.Channel0BreakerFault);
    logData(path + "/channel1BreakerFault", faults.Channel1BreakerFault);
    logData(path + "/channel2BreakerFault", faults.Channel2BreakerFault);
    logData(path + "/channel3BreakerFault", faults.Channel3BreakerFault);
    logData(path + "/channel4BreakerFault", faults.Channel4BreakerFault);
    logData(path + "/channel5BreakerFault", faults.Channel5BreakerFault);
    logData(path + "/channel6BreakerFault", faults.Channel6BreakerFault);
    logData(path + "/channel7BreakerFault", faults.Channel7BreakerFault);
    logData(path + "/channel8BreakerFault", faults.Channel8BreakerFault);
    logData(path + "/channel9BreakerFault", faults.Channel9BreakerFault);
    logData(path + "/channel10BreakerFault", faults.Channel10BreakerFault);
    logData(path + "/channel11BreakerFault", faults.Channel11BreakerFault);
    logData(path + "/channel12BreakerFault", faults.Channel12BreakerFault);
    logData(path + "/channel13BreakerFault", faults.Channel13BreakerFault);
    logData(path + "/channel14BreakerFault", faults.Channel14BreakerFault);
    logData(path + "/channel15BreakerFault", faults.Channel15BreakerFault);
    logData(path + "/channel16BreakerFault", faults.Channel16BreakerFault);
    logData(path + "/channel17BreakerFault", faults.Channel17BreakerFault);
    logData(path + "/channel18BreakerFault", faults.Channel18BreakerFault);
    logData(path + "/channel19BreakerFault", faults.Channel19BreakerFault);
    logData(path + "/channel20BreakerFault", faults.Channel20BreakerFault);
    logData(path + "/channel21BreakerFault", faults.Channel21BreakerFault);
    logData(path + "/channel22BreakerFault", faults.Channel22BreakerFault);
    logData(path + "/channel23BreakerFault", faults.Channel23BreakerFault);
    logData(path + "/hardwareFault", faults.HardwareFault);
  }

  public void logData(String key, Object value) {
    logData(key, value, false);
  }

  public void logData(String key, Object value, boolean once) {
    boolean loggedYet = logEntries.containsKey(key);

    if (!loggedYet) {
      if (value instanceof Boolean)
        logEntries.put(key, instance.getBooleanTopic(key).publish());
      else if (value instanceof Double)
        logEntries.put(key, instance.getDoubleTopic(key).publish());
      else if (value instanceof Short)
        logEntries.put(key, instance.getIntegerTopic(key).publish());
      else if (value instanceof Float)
        logEntries.put(key, instance.getFloatTopic(key).publish());
      else if (value instanceof Integer)
        logEntries.put(key, instance.getIntegerTopic(key).publish());
      else if (value instanceof double[])
        logEntries.put(key, instance.getDoubleArrayTopic(key).publish());
      else if (value instanceof String)
        logEntries.put(key, instance.getStringTopic(key).publish());
      else
        System.out.println("unknown logging data type: " + value.getClass().getSimpleName());
    }

    if (!once || !loggedYet) {
      if (value instanceof Boolean)
        ((BooleanPublisher) logEntries.get(key)).set((boolean) value);
      else if (value instanceof Double)
        ((DoublePublisher) logEntries.get(key)).set((double) value);
      else if (value instanceof Short)
        ((IntegerPublisher) logEntries.get(key)).set(((Short) value).intValue());
      else if (value instanceof Float)
        ((FloatPublisher) logEntries.get(key)).set((float) value);
      else if (value instanceof Integer)
        ((IntegerPublisher) logEntries.get(key)).set((int) value);
      else if (value instanceof double[])
        ((DoubleArrayPublisher) logEntries.get(key)).set((double[]) value);
      else if (value instanceof String)
        ((StringPublisher) logEntries.get(key)).set((String) value);
      else
        System.out.println("unknown logging data type: " + value.getClass().getSimpleName());
    }
  }

  static void REV(REVLibError error, String message) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(String.format("[FAILED] %s: %s", message, error.toString()), false);
      System.out.println(String.format("[FAILED] %s: %s", message, error.toString()));
    }
  }
}
