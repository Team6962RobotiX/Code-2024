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
  int delay = 0;
  LinearFilter filter = LinearFilter.highPass(0.1, 0.02);
  
  public NoteDetector(CANSparkMax motor) {
    this.motor = motor;
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/SPIKE", () -> impulse);
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
    
    if (delay <= 0.1 / 0.02) {
      impulse = 0.0;
      delay++;
      return;
    }

    impulse = filter.calculate(motor.getOutputCurrent() / motor.get());
  }
}
