package frc.robot.util.Logging;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public final class ErrorLogging {
  public static void REV(REVLibError error, String message) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(String.format("[FAILED] %s: %s", message, error.toString()), false);
    }
  }

  public static void CTRE(ErrorCode error, String message) {
    if (error != ErrorCode.OK) {
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
}
