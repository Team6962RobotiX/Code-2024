package frc.robot.subsystems;

import com.github.tommyettinger.colorful.oklab.ColorTools;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class LEDs extends SubsystemBase {
  private static AddressableLED strip;
  private static AddressableLEDBuffer buffer;
  private RobotStateController stateController;
  private static int length = 200;
  private static State state = State.OFF;
  
  public static enum State {
    OFF,
    GREEN,
    DISABLED,
    NO_NOTE,
    DRIVING_TELEOP,
    HAS_NOTE,
    SHOOTING_WARMUP,
    AIMING,
    SHOOTING_SPEAKER,
    HANG,
  }

  public static int[] ANTARES_BLUE = { 36, 46, 68 };
  public static int[] ANTARES_YELLOW = { 255, 100, 0 };
  public static int[] GREEN = { 0, 255, 0 };
  
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
    // state = State.DRIVING_TELEOP;
    switch (state) {
      case OFF:
        setColor(0, length, new int[] {0, 0, 0});
        break;
      case DISABLED:
        //setRainbow(0, length);
        setBumperColorWave(0, length);
        break;
      case DRIVING_TELEOP:
        setBumperColorWave(0, length);
        break;
      case HAS_NOTE:
        setColor(0, length, ANTARES_YELLOW);
        break;
      case AIMING:
        setColorWave(0, length, ANTARES_YELLOW, 2.5); 
        break;
      case SHOOTING_WARMUP:
        setColorWave(0, length, ANTARES_YELLOW, this.stateController.getShooterVelocity() / 250);
        break;
      case GREEN:
        setColor(0, length, GREEN);
        break;
      case NO_NOTE:
        setColor(0, length, new int[] {255, 0, 0});
        break;
      case SHOOTING_SPEAKER:
        setColorFlash(0, length, getBumperColor(), 5);
        break;
      case HANG:
        setTopStripColor(new int[] {255, 100, 0});

        setColorWave(0, Constants.LED.SIDE_STRIP_HEIGHT, ANTARES_BLUE, 1);
        setColorWave(length - Constants.LED.SIDE_STRIP_HEIGHT, length, ANTARES_BLUE, 1);
        break;
    }
    strip.setData(buffer);
    clear();

    state = State.OFF;
  }

  @Override
  public void simulationPeriodic() {
    //setState(State.DISABLED);
  }

  public static Command setStateCommand(State state) {
    return Commands.run(() -> setState(state));
  }

  public static void setState(State state) {
    if (state.ordinal() > LEDs.state.ordinal()) LEDs.state = state;
  }

  private static void setColor(int pixel, int[] RGB) {
    buffer.setRGB(pixel, RGB[0], RGB[1], RGB[2]);
  }

  private static void setColor(int start, int stop, int[] RGB) {
    for (int pixel = start; pixel < stop; pixel++) {
      setColor(pixel, RGB);
    }
  }

  private static void setTopStripColor(int[] RGB) {
    setColor(Constants.LED.SIDE_STRIP_HEIGHT, length - Constants.LED.SIDE_STRIP_HEIGHT, RGB);
  }

  private static void setRainbow(int start, int stop) {
    double time = Timer.getFPGATimestamp();
    for (int pixel = start; pixel < stop; pixel++) {
      int[] rgb = HCLtoRGB(new double[] {(pixel / 100.0 + time * 1.0) % 1.0, 0.2, 0.6});
      setColor(pixel, rgb);
    }
  }

  private static void setColorFlash(int start, int stop, int[] RGB, double speed) {
    double time = Timer.getFPGATimestamp();

    double val = (time * speed) % 1.0;
    if (val < 0.5) {
      setColor(0, length, RGB);
    } else {
      setColor(0, length, new int[] {0, 0, 0});
    }
    
  }

  private static void setColorWave(int start, int stop, int[] RGB, double speed) {
    double time = Timer.getFPGATimestamp();
    for (int pixel = start; pixel < stop; pixel++) {
      double val = (pixel / 200.0 + time * speed) % 1.0;
      if (val < 0.5) {
        setColor(pixel, RGB);
      } else {
        setColor(pixel, new int[] {0, 0, 0});
      }
    }
  }

  private static void setColorWave(int start, int stop, int[] firstRGB, int[] secondRGB, double speed) {
    double time = Timer.getFPGATimestamp();
    for (int pixel = start; pixel < stop; pixel++) {
      double val = (pixel / 200.0 + time * speed) % 1.0;
      if (val < 0.5) {
        setColor(pixel, firstRGB);
      } else {
        setColor(pixel, secondRGB);
      }
    }
  }

  // private static void setGradientWave(int start, int stop, int[] firstRGB, int[] secondRGB, double speed) {
  //   double time = Timer.getFPGATimestamp();
  //   int numLEDs = stop - start;
  //   double maxPoint = (40 + time * 100) % numLEDs; // Pixel index with second rgb

  //   double rDiff = (secondRGB[0] - firstRGB[0]);
  //   double gDiff = (secondRGB[1] - firstRGB[1]);
  //   double bDiff = (secondRGB[2] - firstRGB[2]);

  //   GradientTools.makeGradient(start, stop, numLEDs);

  //   for (int pixel = start; pixel < stop; pixel ++) {
  //     // int r = (int)(firstRGB[0] + rStep * (pixel - start));
  //     // int g = (int)(firstRGB[1] + gStep * (pixel - start));
  //     // int b = (int)(firstRGB[2] + bStep * (pixel - start));
  //     int[] newColor = firstRGB;
  //     newColor[0] = (int) (((newColor[0] + time * 50) * rDiff / 500 + pixel * 10) % 255);
  //     newColor[1] = (int) (((newColor[1] + time * 50) * gDiff / 500 + pixel * 10) % 255);
  //     newColor[2] = (int) (((newColor[2] + time * 50) * bDiff / 500 + pixel * 10) % 255);

  //     setColor(pixel, newColor);
  //   }


  // }

  private static int[] getBumperColor() {
    if (Constants.IS_BLUE_TEAM.get()) {
      return ANTARES_BLUE;  
    } else {
      return new int[] {255, 0, 0};
    }
  }

  private static void setBumperColorWave(int start, int stop) {
    if (Constants.IS_BLUE_TEAM.get()) {
      setColorWave(start, stop, new int[] {23, 127, 255}, new int[] {209, 23, 255}, 2.5);
    } else {
      setColorWave(start, stop, new int[] {255, 0, 0},  getBumperColor(), 2.5);
    } 
  }

  // private static void setBumperGradientWave(int start, int stop) {
  //   if (Constants.IS_BLUE_TEAM) {
  //     setGradientWave(start, stop, getBumperColor(), new int[] {179, 0, 255}, 2.5);
  //   } else {
  //     setGradientWave(start, stop, new int[] {255, 0, 0},  getBumperColor(), 2.5);
  //   }
    
  // }

  //private static void setAcceleratingColorWav

  private static void clear() {
    setColor(0, length, new int[] {0, 0, 0});
  }

  private static int[] HCLtoRGB(double[] HCL) {
    float OKLAB = ColorTools.oklabByHCL((float) HCL[0], (float) HCL[1], (float) HCL[2], (float) 1.0);
    return new int[] {ColorTools.redInt(OKLAB), ColorTools.greenInt(OKLAB), ColorTools.blueInt(OKLAB)};
  }
}
