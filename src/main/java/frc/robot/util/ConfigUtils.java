package frc.robot.util;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public final class ConfigUtils {
  public static void setPeriodicFramePeriods(CANSparkMax sparkMax, int[] statusFramePeriods) {
    configure(List.of(
      () -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, statusFramePeriods[0]),
      () -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, statusFramePeriods[1]),
      () -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, statusFramePeriods[2]),
      () -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, statusFramePeriods[3]),
      () -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, statusFramePeriods[4]),
      () -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, statusFramePeriods[5]),
      () -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, statusFramePeriods[6])
    ));
  }

  public static void configure(Supplier<Object> config) {
    Object status = null;
    for (int i = 0; i < 5; i++) {
      status = config.get();
      if (status instanceof REVLibError) {
        if (status == REVLibError.kOk) {
          return;
        }
      } else if (status instanceof StatusCode) {
        if (status == StatusCode.OK) {
          return;
        }
      } else {
        return;
      }
      DriverStation.reportWarning("[CONFIG WARNING] failed, retrying...", false);
    }
    DriverStation.reportError(String.format("[CONFIG ERROR] %s", status.toString()), false);
  }

  public static void configure(List<Supplier<Object>> configs) {
    for (Supplier<Object> config : configs) {
      configure(config);
    }
  }
}
