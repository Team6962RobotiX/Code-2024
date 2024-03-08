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
  private static int length = 155;
  private static State state = State.OFF;
  
  public static enum State {
    OFF,
    DISABLED,
    DRIVING_TELEOP,
    HAS_VISION_TARGET,
    HAS_NOTE,
    SPIN_UP,
    AIMING,
    BAD,
    SHOOTING,
    AMP,
    HANG,
    GREEN
  }

  public static enum Direction {
    LEFT,
    RIGHT
  }

  public static final int[] WHITE = {255, 255, 255};
  public static final int[] ANTARES_BLUE = { 36, 46, 68 };
  public static final int[] ANTARES_YELLOW = { 255, 100, 0 };
  public static final int[] RED = { 255, 0, 0 };
  public static final int[] RSL_ORANGE = { 255, 100, 0 };
  public static final int[] GREEN = { 0, 255, 0 };
  public static final int[] BLUE = { 23, 127, 255 };
  public static final int[] PURPLE = { 209, 23, 255 };
  
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
    //state = State.SHOOTING;
    switch (state) {
      case OFF:
        setColor(0, length, new int[] {0, 0, 0});
        break;
      case DISABLED:
        //setRainbow(0, length);
        setBumperColorWave(0, length);
        break;
      case HAS_VISION_TARGET:
        setColor(0, length, new int[] {128, 0, 255});
        break;
      case DRIVING_TELEOP:
        setBumperColorWave(0, length);
        break;
      case HAS_NOTE:
        setColor(0, length, ANTARES_YELLOW);
        break;
      case AIMING:
        setColorWave(0, length, ANTARES_YELLOW, 2.5, Direction.LEFT); 
        break;
      case SPIN_UP:
        setColorWave(0, length, ANTARES_YELLOW, this.stateController.getShooterVelocity() / 400, Direction.LEFT);
        break;
      case GREEN:
        setColor(0, length, GREEN);
        break;
      case BAD:
        setColor(0, length, RED);
        break;
      case SHOOTING:
        setColorFlash(0, length, getBumperLEDColor(), 5);
        break;
      
      case AMP:
        setColor(0, length, WHITE);
        setTopStripColor(PURPLE);
        //  setColorWave(int start, int stop, int[] firstRGB, int[] secondRGB, double speed, Direction dir)
        setColorWave(0, Constants.LED.SIDE_STRIP_HEIGHT, PURPLE, WHITE, 1, Direction.RIGHT);
        setColorWave(length - Constants.LED.SIDE_STRIP_HEIGHT, length, PURPLE, WHITE, 1, Direction.LEFT);
        break;
      case HANG:
        setColor(0, length, WHITE);
        setTopStripColor(RED);
        setColorWave(0, Constants.LED.SIDE_STRIP_HEIGHT, RED, WHITE, 1, Direction.RIGHT);
        setColorWave(length - Constants.LED.SIDE_STRIP_HEIGHT, length, RED, WHITE, 1, Direction.LEFT);
        //setColor(150, PURPLE);
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
// setColorWave(length - Constants.LED.SIDE_STRIP_HEIGHT, length, RSL_ORANGE, 1, Direction.RIGHT);
  private static void setColorWave(int start, int stop, int[] RGB, double speed, Direction dir) {
    double time = Timer.getFPGATimestamp();
    
    for (int pixel = 0; pixel < stop - start; pixel++) {
      double val = (pixel / 50.0 + time * speed) % 1.0;
      int p = pixel + start;
      if (val < 0.5) {
        if (dir == Direction.LEFT) {
          setColor(p, RGB);
        } else if (dir == Direction.RIGHT) {
          setColor(stop - pixel - 1, RGB);
        }
      } else {
        if (dir == Direction.LEFT) {
          setColor(p, new int[] {0, 0, 0});
        } else if (dir == Direction.RIGHT) {
          setColor(stop - pixel - 1, new int[] {0, 0, 0});
        }

      }
    }
  }

  private static void setColorWave(int start, int stop, int[] firstRGB, int[] secondRGB, double speed, Direction dir) {
    double time = Timer.getFPGATimestamp();
    for (int pixel = 0; pixel < stop - start; pixel++) {
      double val = (pixel / 50.0 + time * speed) % 1.0;
      int p = pixel + start;
      if (val < 0.5) {
        if (dir == Direction.LEFT) {
          setColor(p, firstRGB);
        } else if (dir == Direction.RIGHT) {
          setColor(stop - pixel - 1, firstRGB);
        }
      } else {
        if (dir == Direction.LEFT) {
          setColor(p, secondRGB);
        } else if (dir == Direction.RIGHT) {
          setColor(stop - pixel - 1, secondRGB);
        }

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
      return RED;
    }
  }

  private static int[] getBumperLEDColor () {
    if (Constants.IS_BLUE_TEAM.get()) {
      return BLUE;  
    } else {
      return RED;
    }
  }

  private static void setBumperColorWave(int start, int stop) {
    if (Constants.IS_BLUE_TEAM.get()) {
      setColorWave(start, stop, getBumperLEDColor(), PURPLE, 2.5, Direction.LEFT);
    } else {
      setColorWave(start, stop, getBumperLEDColor(),  ANTARES_YELLOW, 2.5, Direction.LEFT);
    } 
  }

  // private static void setBumperGradientWave(int start, int stop) {
  //   if (Constants.IS_BLUE_TEAM) {
  //     setGradientWave(start, stop, getBumperColor(), new int[] {179, 0, 255}, 2.5);
  //   } else {
  //     setGradientWave(start, stop, RED,  getBumperColor(), 2.5);
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
