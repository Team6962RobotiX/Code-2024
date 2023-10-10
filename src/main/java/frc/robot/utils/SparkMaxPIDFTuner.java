package frc.robot.utils;

import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxPIDFTuner extends SubsystemBase {
  private double kP, kI, kD, kFF;
  private GenericEntry kP_entry, kI_entry, kD_entry, kFF_entry, setpoint_entry, measurement_entry, graph_entry;
  private Supplier<Double> setpointSupplier, measurementSupplier;
  private SparkMaxPIDController PIDF;
  private SparkMaxPIDController[] followers;
  private ShuffleboardTab tab;

  public SparkMaxPIDFTuner(String name, SparkMaxPIDController PIDF, Supplier<Double> setpointSupplier,
      Supplier<Double> measurementSupplier) {
    this(name, PIDF, new SparkMaxPIDController[] {}, setpointSupplier, measurementSupplier);
  }

  public SparkMaxPIDFTuner(String name, SparkMaxPIDController PIDF, SparkMaxPIDController[] followers,
      Supplier<Double> setpointSupplier, Supplier<Double> measurementSupplier) {
    this.PIDF = PIDF;
    this.followers = followers;
    this.setpointSupplier = setpointSupplier;
    this.measurementSupplier = measurementSupplier;
    kP = PIDF.getP();
    kI = PIDF.getI();
    kD = PIDF.getD();
    kFF = PIDF.getFF();
    tab = Shuffleboard.getTab(name + " PID Tuner");
    kP_entry = tab.add(name + " kP", kP)
        .withPosition(0, 0)
        .getEntry();
    kI_entry = tab.add(name + " kI", kI)
        .withPosition(1, 0)
        .getEntry();
    kD_entry = tab.add(name + " kD", kD)
        .withPosition(2, 0)
        .getEntry();
    kFF_entry = tab.add(name + " kFF", kFF)
        .withPosition(3, 0)
        .getEntry();

    setpoint_entry = tab.add(name + " setpoint", setpointSupplier.get()).getEntry();
    measurement_entry = tab.add(name + " measurement", measurementSupplier.get()).getEntry();
    graph_entry = tab.add(name + " graph", new Double[] { setpointSupplier.get(), measurementSupplier.get() })
        .withWidget(BuiltInWidgets.kGraph)
        .withPosition(0, 3)
        .withWidget(BuiltInWidgets.kGraph)
        .withSize(3, 2)
        .getEntry();
  }

  @Override
  public void periodic() {
    double new_kP = kP_entry.getDouble(0);
    double new_kI = kI_entry.getDouble(0);
    double new_kD = kD_entry.getDouble(0);
    double new_kFF = kFF_entry.getDouble(0);

    setpoint_entry.setDouble(setpointSupplier.get());
    measurement_entry.setDouble(measurementSupplier.get());
    graph_entry.setDoubleArray(new Double[] { setpointSupplier.get(), measurementSupplier.get() });

    if (new_kP != kP) {
      kP = new_kP;
      Logger.REV(PIDF.setP(kP), "PIDF.setP");
      for (SparkMaxPIDController follower : followers)
        Logger.REV(follower.setP(kP), "follower.setP");
    }
    if (new_kI != kI) {
      kI = new_kI;
      Logger.REV(PIDF.setI(kI), "PIDF.setI");
      for (SparkMaxPIDController follower : followers)
        Logger.REV(follower.setI(kP), "follower.setI");
    }
    if (new_kD != kD) {
      kP = new_kD;
      Logger.REV(PIDF.setD(kD), "PIDF.setD");
      for (SparkMaxPIDController follower : followers)
        Logger.REV(follower.setD(kP), "follower.setD");
    }
    if (new_kFF != kFF) {
      kFF = new_kFF;
      Logger.REV(PIDF.setFF(kFF), "PIDF.setFF");
      for (SparkMaxPIDController follower : followers)
        Logger.REV(follower.setFF(kP), "follower.setFF");
    }
  }
}
