package frc.robot.util;

import java.util.List;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.NEO;
import frc.robot.util.Logging.Logger;

public final class SparkMaxUtils {
  public void setPeriodicFramePeriods(CANSparkMax sparkMax, int[] statusFramePeriods) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, statusFramePeriods[0]);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, statusFramePeriods[1]);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, statusFramePeriods[2]);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, statusFramePeriods[3]);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, statusFramePeriods[4]);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, statusFramePeriods[5]);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, statusFramePeriods[6]);
  }

  public static void configure(CANSparkMax sparkMax, Supplier<REVLibError> config) {
    REVLibError status = REVLibError.kOk;
    for (int i = 0; i < 5; i++) {
      status = config.get();
      if (status == REVLibError.kOk) {
        return;
      }
    }
    DriverStation.reportError(String.format("SparkMAX %s, REASON: %s", sparkMax.getDeviceId(), status.toString()), false);
  }

  public static void configure(CANSparkMax sparkMax, List<Supplier<REVLibError>> configs) {
    for (Supplier<REVLibError> config : configs) {
      configure(sparkMax, config);
    }
  }
}
