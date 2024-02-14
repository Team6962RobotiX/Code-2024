package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class StatusChecks {
  private static ShuffleboardTab tab = Shuffleboard.getTab("Status Checks");

  public static void addCheck(String name, BooleanSupplier supplier) {
    tab.addBoolean(name, supplier).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1);
  }
}