package frc.robot.util.software.Logging;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatusChecks {
  public static int row = 0;
  public static int column = 0;

  private static void addCheck(String name, BooleanSupplier supplier) {
    ShuffleboardTab tab = Shuffleboard.getTab("Status Checks");
    tab.addBoolean(name.replace("/", " "), supplier).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1).withPosition(column, row);
    column++;
    if (column > 10) {
      column = 0;
      row++;
    }
  }

  public static void addCheck(SubsystemBase subsystem, String name, BooleanSupplier supplier) {
    addCheck(subsystem.getClass().getSimpleName() + "/" + name, supplier);
  }
}