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

import javax.print.attribute.standard.Media;

public class NoteDetector extends SubsystemBase {
  int filterSize = 10;
  MedianFilter filter = new MedianFilter(filterSize);
  double delay = NEO.SAFE_RAMP_RATE * 3.0;
  double delayCounter = 0.0;
  CANSparkMax motor;
  double gearing = 0.0;
  double filteredTorque;
  double freeTorque;
  boolean hasNote = false;

  public NoteDetector(CANSparkMax motor, double gearing, double freeTorque) {
    this.motor = motor;
    this.gearing = gearing;
    this.freeTorque = freeTorque;
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/hasNote", () -> hasNote());
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/appliedTorque", () -> filteredTorque);
  }

  @Override
  public void periodic() {
    double output = motor.get();
    
    if (output == 0.0) {
      delayCounter = 0.0;
      filter.reset();
      return;
    }

    if (delayCounter <= delay) {
      delayCounter += 0.02;
      filter.reset();
      return;
    }

    double motorTorque = NEO.STATS.stallTorqueNewtonMeters / NEO.STATS.stallCurrentAmps * motor.getOutputCurrent();
    double appliedTorque = motorTorque * gearing;
    filteredTorque = filter.calculate(appliedTorque);

    if (filteredTorque - freeTorque * 1.5 > 0) {
      hasNote = true;
    } else {
      hasNote = false;
    }
  }

  public boolean hasNote() {
    return hasNote;
  }
}
