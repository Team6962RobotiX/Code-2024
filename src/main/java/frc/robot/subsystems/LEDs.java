package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.Color;

import com.github.tommyettinger.colorful.oklab.ColorTools;

public class LEDs extends SubsystemBase {
  private static AddressableLED strip;
  private static AddressableLEDBuffer buffer;
  private RobotStateController stateController;
  private static int length = 200;
  
  public LEDs(RobotStateController stateController) {
    this.stateController = stateController;
    strip = new AddressableLED(1);
    buffer = new AddressableLEDBuffer(length);
    strip.setLength(buffer.getLength());

    strip.setData(buffer);
    strip.start();
  }

  @Override
  public void periodic() {
    if (stateController.hasNote()) {
      setColor(0, length, new int[] {255, 87, 3});
    } else {
      setColor(0, length, new int[] {2, 21, 61});
    }    
    //setRainbow(0, length);
    strip.setData(buffer);
    clear();
  }

  @Override
  public void simulationPeriodic() {

  }

  public static void setColor(int pixel, int[] RGB) {
    buffer.setRGB(pixel, RGB[0], RGB[1], RGB[2]);
  }

  public static void setColor(int start, int stop, int[] RGB) {
    for (int pixel = start; pixel < stop; pixel++) {
      setColor(pixel, RGB);
    }
  }

  public static void setRainbow(int start, int stop) {
    double time = Timer.getFPGATimestamp();
    for (int pixel = start; pixel < stop; pixel++) {
      setColor(pixel, HCLtoRGB(new double[] {(pixel / 100.0 + time * 1.0) % 1.0, 1.0, 0.5}));
    }
  }

  public static void setColorWave(int start, int stop, int[] RGB) {
    double time = Timer.getFPGATimestamp();
    for (int pixel = start; pixel < stop; pixel++) {
      double val = (pixel / 200.0 + time * 2.5) % 1.0;
      if (val < 0.5) {
        setColor(pixel, RGB);
      } else {
        setColor(pixel, new int[] {0, 0, 0});
      }
    }
  }


  public static void clear() {
    setColor(0, length, new int[] {0, 0, 0});
  }

  public static int[] HCLtoRGB(double[] HCL) {
    float OKLAB = ColorTools.oklabByHCL((float) HCL[0], (float) HCL[1], (float) HCL[2], (float) 1.0);
    return new int[] {ColorTools.redInt(OKLAB), ColorTools.greenInt(OKLAB), ColorTools.blueInt(OKLAB)};
  }
}
