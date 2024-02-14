package frc.robot.subsystems.notes;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteDetector extends SubsystemBase {
  double currentThreshold;
  CANSparkMax motor;
  Debouncer debouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);
  MedianFilter medianFilter = new MedianFilter(5);
  boolean hasNote = false;
  
  public NoteDetector(CANSparkMax motor, double currentThreshold) {
    this.currentThreshold = currentThreshold;
    this.motor = motor;
  }

  public boolean hasNote() {
    return hasNote;
  }

  @Override
  public void periodic() {
    double filteredCurrent = medianFilter.calculate(motor.getOutputCurrent());
    hasNote = debouncer.calculate(filteredCurrent > currentThreshold) || hasNote;
  }
}
