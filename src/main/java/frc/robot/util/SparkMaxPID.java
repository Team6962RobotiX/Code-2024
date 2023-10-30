package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

public class SparkMaxPID {
  public SparkMax sparkMax;
  public SparkMaxPIDController controller;
  public boolean smartMotionEnabled = false;

  public SparkMaxPID(SparkMax sparkMax) {
    this.sparkMax = sparkMax;
    this.controller = ((CANSparkMax) sparkMax).getPIDController();
    controller.setFeedbackDevice(((CANSparkMax) sparkMax).getEncoder());
  }

  public void setPID(double P, double I, double D) {
    sparkMax.execute(() -> controller.setP(P), "controller.setP");
    sparkMax.execute(() -> controller.setI(I), "controller.setI");
    sparkMax.execute(() -> controller.setD(D), "controller.setD");
  }

  public void setPIDF(double P, double I, double D, double FF) {
    sparkMax.execute(() -> controller.setFF(FF), "controller.setFF");
    setPID(P, I, D);
  }

  public void enableWrap(double minInput, double maxInput) {
    sparkMax.execute(() -> controller.setPositionPIDWrappingEnabled(true), "controller.setPositionPIDWrappingEnabled");
    sparkMax.execute(() -> controller.setPositionPIDWrappingMinInput(minInput), "controller.setPositionPIDWrappingMinInput");
    sparkMax.execute(() -> controller.setPositionPIDWrappingMaxInput(maxInput), "controller.setPositionPIDWrappingMaxInput");
  }

  public void enableSmartMotion(double maxVelocity, double maxAcceleration) {
    smartMotionEnabled = true;
    sparkMax.execute(() -> controller.setSmartMotionMaxVelocity(maxVelocity, 0), "controller.setSmartMotionMaxVelocity");
    sparkMax.execute(() -> controller.setSmartMotionMaxAccel(maxAcceleration, 0), "controller.setSmartMotionMaxAccel");
  }

  public void setMotorPowerCap(double cap) {
    sparkMax.execute(() -> controller.setOutputRange(-cap, cap, 0), "controller.setOutputRange");
  }

  public void setSmartMotionAccelStrategy(AccelStrategy accelStrategy) {
    sparkMax.execute(() -> controller.setSmartMotionAccelStrategy(accelStrategy, 0), "controller.setSmartMotionAccelStrategy");
  }

  public void setPosition(double position) {
    setPosition(position, 0);
  }

  public void setPosition(double position, double arbitraryFF) {
    if (smartMotionEnabled) {
      controller.setReference(position, ControlType.kSmartMotion, 0, arbitraryFF);
    } else {
      controller.setReference(position, ControlType.kPosition, 0, arbitraryFF);
    }
  }

  public void setVelocity(double velocity) {
    setPosition(velocity, 0);
  }

  public void setVelocity(double velocity, double arbitraryFF) {
    if (smartMotionEnabled) {
      controller.setReference(velocity, ControlType.kSmartVelocity, 0, arbitraryFF);
    } else {
      controller.setReference(velocity, ControlType.kVelocity, 0, arbitraryFF);
    }
  }
}