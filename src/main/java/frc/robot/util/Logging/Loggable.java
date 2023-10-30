package frc.robot.util.Logging;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.LOGGING;

public class Loggable {
  private String path;
  private Supplier<Object> supplier;
  
  public Loggable(String path, Supplier<Object> supplier) {
    this.path = path;
    this.supplier = supplier;
  }

  public void logTo(NetworkTable table) {
    table.getEntry(path).setValue(supplier.get());
  }
}
