package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatusChecks extends SubsystemBase {
  private static ShuffleboardTab tab = Shuffleboard.getTab("Status Checks");

  public static void addCheck(String name, BooleanSupplier supplier) {
    tab.addBoolean(name, supplier).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1);
  }
}