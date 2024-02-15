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
  double totalDelay = 0.1 / 0.02;
  int delay = 0;
  LinearFilter highPass = LinearFilter.highPass(0.1, 0.02);
  LinearFilter movingAverage = LinearFilter.movingAverage(10);
  double scalingFactor = 1.0;
  
  public NoteDetector(CANSparkMax motor, boolean isNeo550) {
    this.motor = motor;
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/impulse", () -> impulse);
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/scaledCurrent", () -> scaledCurrent);
    if (isNeo550) {
      scalingFactor = 5.0;
    }
  }

  public boolean hasJustReceivedNote() {
    return impulse > Presets.NOTE_DETECTION_IMPULSE && delay > totalDelay;
  }

  public boolean hasJustReleaseddNote() {
    return impulse < -Presets.NOTE_DETECTION_IMPULSE && delay > totalDelay;
  }

  @Override
  public void periodic() {
    double output = motor.get();
    scaledCurrent = 0.0;

    if (output == 0.0) {
      impulse = 0.0;
      delay = 0;
      movingAverage.reset();
      return;
    }

    scaledCurrent = movingAverage.calculate((motor.getOutputCurrent() / Math.abs(motor.get())) * scalingFactor);
    impulse = highPass.calculate(scaledCurrent);

    if (delay <= totalDelay) {
      delay++;
      movingAverage.reset();
      return;
    }
  }
}
