//Based on FRC#6328 software
package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.PolynomialRegression;

public class FeedForwardCharacterization extends CommandBase {
  private static final double startDelaySecs = 2.0;
  private static final double rampRateVoltsPerSec = 0.25;
  private final boolean forwards;
  private final FeedForwardCharacterizationData data;
  private final Consumer<Double> voltageConsumer;
  private final Supplier<Double> velocitySupplierPrimary;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization for a drive. */
  public FeedForwardCharacterization(Subsystem drive, boolean forwards,
      Consumer<Double> voltageConsumer,
      Supplier<Double> leftVelocitySupplier,
      Supplier<Double> rightVelocitySupplier) {
    addRequirements(drive);
    this.forwards = forwards;
    this.data = new FeedForwardCharacterizationData("data");
    this.voltageConsumer = voltageConsumer;
    this.velocitySupplierPrimary = leftVelocitySupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < startDelaySecs) {
      voltageConsumer.accept(0.0);
    } else {
      double voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec * (forwards ? 1 : -1);
      voltageConsumer.accept(voltage);
      data.add(velocitySupplierPrimary.get(), voltage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0);
    timer.stop();
    data.print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class FeedForwardCharacterizationData {
    private final String name;
    private final List<Double> velocityData = new ArrayList<>();
    private final List<Double> voltageData = new ArrayList<>();

    public FeedForwardCharacterizationData(String name) {
      this.name = name;
    }

    public void add(double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void print() {
      PolynomialRegression regression = new PolynomialRegression(
          velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
          voltageData.stream().mapToDouble(Double::doubleValue).toArray(), 1);

      System.out.println("FF Characterization Results (" + name + "):");
      System.out
          .println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tR2=%.5f", regression.R2()));
      System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
      System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
    }
  }
}