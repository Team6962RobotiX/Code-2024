package frc.robot.util.hardware;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Presets;
import frc.robot.util.software.Logging.Logger;

/**
 * Used for detecting notes based on current draw from a motor
 * Cannot be used in closed loop control
 */
public class NoteDetector extends SubsystemBase {
  CANSparkMax motor;
  double impulse = 0.0;
  double scaledCurrent = 0.0;
  int delay = 0;
  LinearFilter highPass = LinearFilter.highPass(0.1, 0.02);
  MedianFilter medianFilter = new MedianFilter(5);
  double scalingFactor = 1.0;
  
  public NoteDetector(CANSparkMax motor, boolean isNeo550) {
    this.motor = motor;
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/impulse", () -> impulse);
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/scaledCurrent", () -> scaledCurrent);
    if (isNeo550) {
      scalingFactor = 2.5;
    }
  }

  public boolean hasJustReceivedNote() {
    return impulse > Presets.NOTE_DETECTION_IMPULSE;
  }

  public boolean hasJustReleaseddNote() {
    return impulse < Presets.NOTE_DETECTION_IMPULSE;
  }

  @Override
  public void periodic() {
    double output = motor.get();
    if (output == 0.0) {
      impulse = 0.0;
      delay = 0;
      return;
    }
    
    if (delay <= 0.25 / 0.02) {
      impulse = 0.0;
      delay++;
      return;
    }

    scaledCurrent = medianFilter.calculate((motor.getOutputCurrent() / motor.getAppliedOutput()) / scalingFactor);
    impulse = highPass.calculate(scaledCurrent);
  }
}
