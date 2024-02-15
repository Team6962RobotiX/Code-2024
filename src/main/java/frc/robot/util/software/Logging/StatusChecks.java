package frc.robot.util.software.Logging;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatusChecks {
  private static ShuffleboardTab tab = Shuffleboard.getTab("Status Checks");

  public static void addCheck(String name, BooleanSupplier supplier) {
    tab.addBoolean(name.replace("/", " "), supplier).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1);
  }

  public static void addCheck(SubsystemBase subsystem, String name, BooleanSupplier supplier) {
    addCheck(subsystem.getClass().getSimpleName() + "/" + name, supplier);
  }
}