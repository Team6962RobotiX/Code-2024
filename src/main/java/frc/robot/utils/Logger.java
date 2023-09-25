package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.hal.PowerDistributionVersion;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.subsystems.*;

import frc.robot.Constants.*;

public class Logger {
  DataLog log;
  Map<String, Object> logEntries = new HashMap<String, Object>();
  SwerveDrive drive;
  PowerDistribution PDP = new PowerDistribution(CAN.PDP, ModuleType.kRev);
  public int loopCount = 0;
  public boolean inTeleop = false;

  public Logger(SwerveDrive drive) {
    this.drive = drive;
    DataLogManager.start();
  }

  public void logAll() {
    if (!inTeleop) {
      return;
    }
    log = DataLogManager.getLog();
    if (EnabledLogging.ENABLE_DRIVE)
      logSwerve("/swerveDrive", drive);
    if (EnabledLogging.ENABLE_PDP)
      logPDP("/powerDistribution", PDP);
    if (EnabledLogging.ENABLE_ROBOT_CONTROLLER)
      logRobotController("/robotController");
    if (EnabledLogging.ENABLE_DRIVER_STATION)
      logDriverStation("/driverStation");
  }

  public void logDriverStation(String path) {
    DriverStation.startDataLog(log, true);
  }

  public void logSwerve(String path, SwerveDrive drive) {
    logNavX(path + "/gyro", drive.getGyro());
    logOdometer(path + "/odometer", drive.getOdometer());
    logData(path + "/heading", drive.getHeading());
    logData(path + "/totalCurrent", drive.getCurrent());
    logData(path + "/totalVoltage", drive.getVoltage());
    logPose(path + "/pose", drive.getPose());
    logModuleStates(path + "/moduleStates", drive.getTargetModuleStates(), drive.getMeasuredModuleStates(), drive.getModulePositions());

    for (SwerveModule module : drive.getModules()) {
      logSwerveModule(path + "/modules/" + module.getName(), module);
    }
  }

  public void logSwerveModule(String path, SwerveModule module) {
    logSparkMax(path + "/driveMotor", module.getDriveMotor());
    logSparkMax(path + "/steerMotor", module.getSteerMotor());
    logCANCoder(path + "/canCoder", module.getCanCoder());
    logData(path + "/steerDegrees", module.getSteerDegrees());
    logData(path + "/steerRadians", module.getSteerRadians());
    logData(path + "/velocity", module.getVelocity());
    logData(path + "/distanceTraveled", module.getDistanceTraveled());
  }

  public void logSparkMax(String path, CANSparkMax sparkMax) {
    logData(path + "/power", sparkMax.get());
    logData(path + "/appliedOutputDutyCycle", sparkMax.getAppliedOutput());
    logData(path + "/busVoltage", sparkMax.getBusVoltage());
    logData(path + "/motorTemperature", sparkMax.getMotorTemperature());
    logData(path + "/outputCurrent", sparkMax.getOutputCurrent());
    logData(path + "/faults", sparkMax.getFaults());
    logData(path + "/closedLoopRampRate", sparkMax.getClosedLoopRampRate(), true);
    logData(path + "/openLoopRampRate", sparkMax.getOpenLoopRampRate(), true);
    logData(path + "/deviceId", sparkMax.getDeviceId(), true);
    logData(path + "/firmwareString", sparkMax.getFirmwareString(), true);
    logData(path + "/voltageCompensationNominalVoltage", sparkMax.getVoltageCompensationNominalVoltage(), true);
    logData(path + "/firmwareVersion", sparkMax.getFirmwareVersion(), true);
    logData(path + "/stickyFaults", sparkMax.getStickyFaults(), true);
    logRelativeEncoder(path + "/relativeEncoder", sparkMax.getEncoder());
  }

  public void logRelativeEncoder(String path, RelativeEncoder encoder) {
    logData(path + "/position", encoder.getPosition());
    logData(path + "/velocity", encoder.getVelocity());
    logData(path + "/averageSamplingDepth", encoder.getAverageDepth(), true);
    logData(path + "/countsPerRevolution", encoder.getCountsPerRevolution(), true);
    logData(path + "/isInverted", encoder.getInverted(), true);
    logData(path + "/positionMeasurementPeriod", encoder.getMeasurementPeriod(), true);
    logData(path + "/positionConversionFactor", encoder.getPositionConversionFactor(), true);
    logData(path + "/velocityConversionFactor", encoder.getVelocityConversionFactor(), true);
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
    logData(path + "/odometer", new double[] {
        odometer.getPoseMeters().getX(),
        odometer.getPoseMeters().getY(),
        odometer.getPoseMeters().getRotation().getRadians()
    });
  }

  public void logPose(String path, Pose2d pose) {
    logData(path, new double[] {
        pose.getX(),
        pose.getY(),
        pose.getRotation().getRadians()
    });
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

  public void logPDP(String path, PowerDistribution PDP) {
    logPDPFaults(path + "/faults", PDP.getFaults());

    logData(path + "/canId", PDP.getModule(), true);
    for (int i = 0; i <= 23; i++) {
      logData(path + "/channels/channel" + i + "Current", PDP.getCurrent(i));
    }
    logData(path + "/isSwitchableChannelOn", PDP.getSwitchableChannel(), true);
    logData(path + "/temperature", PDP.getTemperature());
    logData(path + "/totalCurrent", PDP.getTotalCurrent());
    logData(path + "/totalJoules", PDP.getTotalEnergy());
    logData(path + "/totalWatts", PDP.getTotalPower());
    logData(path + "/voltage", PDP.getVoltage());
  }

  public void logPDPFaults(String path, PowerDistributionFaults faults) {
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
        logEntries.put(key, new BooleanLogEntry(log, key));
      else if (value instanceof Double)
        logEntries.put(key, new DoubleLogEntry(log, key));
      else if (value instanceof Short)
        logEntries.put(key, new IntegerLogEntry(log, key));
      else if (value instanceof Float)
        logEntries.put(key, new FloatLogEntry(log, key));
      else if (value instanceof Integer)
        logEntries.put(key, new IntegerLogEntry(log, key));
      else if (value instanceof double[])
        logEntries.put(key, new DoubleArrayLogEntry(log, key));
      else if (value instanceof String)
        logEntries.put(key, new StringLogEntry(log, key));
      else
        System.out.println("unknown logging data type: " + value.getClass().getSimpleName());
    }

    if (!once || !loggedYet) {
      if (value instanceof Boolean)
        ((BooleanLogEntry) logEntries.get(key)).append((boolean) value);
      else if (value instanceof Double)
        ((DoubleLogEntry) logEntries.get(key)).append((double) value);
      else if (value instanceof Short)
        ((IntegerLogEntry) logEntries.get(key)).append(((Short) value).intValue());
      else if (value instanceof Float)
        ((FloatLogEntry) logEntries.get(key)).append((float) value);
      else if (value instanceof Integer)
        ((IntegerLogEntry) logEntries.get(key)).append((int) value);
      else if (value instanceof double[])
        ((DoubleArrayLogEntry) logEntries.get(key)).append((double[]) value);
      else if (value instanceof String)
        ((StringLogEntry) logEntries.get(key)).append((String) value);
      else
        System.out.println("unknown logging data type: " + value.getClass().getSimpleName());
    }
  }
}
