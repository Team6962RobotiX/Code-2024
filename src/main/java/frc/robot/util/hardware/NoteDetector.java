package frc.robot.util.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Field;
import frc.robot.Presets;
import frc.robot.Constants.NEO;
import frc.robot.util.software.Logging.Logger;

import java.util.ArrayDeque;
import java.util.Queue;

public class NoteDetector extends SubsystemBase {
  int scope = 10;
  Queue<Double> lastReadings = new ArrayDeque<>(scope * 2);
  double delay = NEO.SAFE_RAMP_RATE * 3.0;
  double delayCounter = 0.0;
  CANSparkMax motor;
  RelativeEncoder encoder;
  double encoderOffset = 0.0;
  double gearing = 0.0;
  double radius = 0.0;

  public NoteDetector(CANSparkMax motor, double gearing, double radius) {
    this.motor = motor;
    this.gearing = gearing;
    this.radius = radius;
    encoder = motor.getEncoder();
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/currentChangeRatio", () -> getChangeRatio());
  }

  @Override
  public void periodic() {
    double output = motor.get();
    
    if (output == 0.0) {
      delayCounter = 0.0;
      lastReadings = new ArrayDeque<>(scope * 2);
      return;
    }

    if (delayCounter <= delay) {
      delayCounter += 0.02;
      lastReadings = new ArrayDeque<>(scope * 2);
      return;
    }
    
    updateLastReadings(motor.getOutputCurrent());
  }
  
  private void updateLastReadings(double current) {
    if (lastReadings.size() == scope * 2) {
      lastReadings.poll();
    }
    lastReadings.offer(current);
  }
  
  private double getChangeRatio() {
    if (lastReadings.size() != scope * 2) {
      return 0.0;
    }
    
    double oldAverage = 0, newAverage = 0;
    int i = 0;
    for (double reading : lastReadings) {
      if (i < scope) {
        oldAverage += reading;
      } else {
        newAverage += reading;
      }
      i++;
    }

    oldAverage /= (double) scope;
    newAverage /= (double) scope;
    
    if (oldAverage == 0.0 || newAverage == 0.0) {
      return 0.0;
    }
    
    if (newAverage > oldAverage) {
      return newAverage / oldAverage - 1.0;
    } else {
      return 1.0 - (oldAverage / newAverage);
    }
  }

  public boolean hasJustReleasedNote() {
    return getChangeRatio() > Presets.NOTE_DETECTION_THRESHOLD;
  }

  public boolean hasJustReceivedNote() {
    return getChangeRatio() < Presets.NOTE_DETECTION_THRESHOLD;
  }
}
