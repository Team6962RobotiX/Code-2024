package frc.robot.util;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Presets;

public class NoteDetector {
  CANSparkMax motor;
  double impulse = 0.0;
  LinearFilter filter = LinearFilter.highPass(0.1, 0.02);
  
  public NoteDetector(CANSparkMax motor) {
    this.motor = motor;
  }

  public boolean hasJustReceivedNote() {
    return impulse > Presets.NOTE_DETECTION_IMPULSE;
  }

  public boolean hasJustReleaseddNote() {
    return impulse < Presets.NOTE_DETECTION_IMPULSE;
  }

  public void run() {
    double impulse = filter.calculate(motor.getOutputCurrent() / motor.getAppliedOutput());
  }
}
