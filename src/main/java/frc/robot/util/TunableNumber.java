package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TunableNumber extends SubsystemBase {
  private String name;
  private Consumer<Double> setter;
  private Supplier<Double> getter;
  private GenericEntry entry;
  private ShuffleboardTab tab;

  public TunableNumber(String name, Supplier<Double> getter, Consumer<Double> setter) {
    this.name = name;
    this.setter = setter;
    tab = Shuffleboard.getTab("Tunable Numbers");
    entry = tab.add(name, getter.get())
      .withWidget(BuiltInWidgets.kNumberSlider)
      .getEntry();
  }

  @Override
  public void periodic() {
    double new_n = entry.getDouble(0);

    if (new_n != getter.get()) {
      setter.accept(new_n);
    }
  }
}