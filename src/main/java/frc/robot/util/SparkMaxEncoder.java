package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import frc.robot.util.Logging.Logger;

public class SparkMaxEncoder implements RelativeEncoder {
  public RelativeEncoder encoder;
  public SparkMax sparkMax;

  public SparkMaxEncoder(RelativeEncoder encoder, SparkMax sparkMax) {
    this.encoder = encoder;
    this.sparkMax = sparkMax;
  }

  public double getPosition() {
    double position = encoder.getPosition();
    Logger.log(sparkMax.getLogPath() + "/calls/encoder.getPosition", position);
    return position;
  }

  public double getVelocity() {
    double velocity = encoder.getVelocity();
    Logger.log(sparkMax.getLogPath() + "/calls/encoder.getVelocity", velocity);
    return velocity;
  }

  public REVLibError setPosition(double position) {
    return sparkMax.execute(() -> encoder.setPosition(position), "encoder.setPosition");
  }

  public REVLibError setPositionConversionFactor(double factor) {
    return sparkMax.execute(() -> encoder.setPositionConversionFactor(factor), "encoder.setPositionConversionFactor");
  }

  public REVLibError setVelocityConversionFactor(double factor) {
    return sparkMax.execute(() -> encoder.setVelocityConversionFactor(factor), "encoder.setVelocityConversionFactor");
  }

  public double getPositionConversionFactor() {
    return encoder.getPositionConversionFactor();
  }

  public double getVelocityConversionFactor() {
    return encoder.getVelocityConversionFactor();
  }

  public REVLibError setAverageDepth(int depth) {
    return sparkMax.execute(() -> encoder.setAverageDepth(depth), "encoder.setAverageDepth");
  }

  public int getAverageDepth() {
    return encoder.getAverageDepth();
  }

  public REVLibError setMeasurementPeriod(int period_ms) {
    return sparkMax.execute(() -> encoder.setMeasurementPeriod(period_ms), "encoder.setMeasurementPeriod");
  }

  public int getMeasurementPeriod() {
    return encoder.getMeasurementPeriod();
  }

  public int getCountsPerRevolution() {
    return encoder.getCountsPerRevolution();
  }

  public REVLibError setInverted(boolean inverted) {
    return sparkMax.execute(() -> encoder.setInverted(inverted), "encoder.setInverted");
  }

  public boolean getInverted() {
    return encoder.getInverted();
  }
}
