// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.kauailabs.navx.frc.AHRS;

public final class SelfCheck {

  public static PowerDistribution PDP = new PowerDistribution(CAN.PDP, ModuleType.kRev);

  public static void checkMotorFaults(CANSparkMax[] motorsControllers) {
    for (CANSparkMax motorController : motorsControllers) {
      int faultFlags = motorController.getFaults();

      if (faultFlags != 0) {
        for (MotorFault fault : MotorFault.values()) {
          boolean hasFault = (faultFlags & fault.getValue()) != 0;

          if (hasFault) {
            switch (fault) {
            case kBrownout:
              warn("Spark Max " + motorController.getDeviceId() + " is having a brownout");
              break;

            case kCanMismatch:
              warn("Spark Max " + motorController.getDeviceId() + " is having a CAN ID mismatch");
              break;

            case kStall:
              warn("Spark Max " + motorController.getDeviceId() + " is having a stall");
              break;

            case kSupplyVoltage:
              warn("Spark Max " + motorController.getDeviceId() + " is having a supply voltage issue");
              break;

            default:
              warn("Spark Max " + motorController.getDeviceId() + " is having a fault: " + fault.name());
              break;
            }
          }
        }
      }
    }
  }

  public static void checkPDPFaults() {
    PowerDistributionFaults faults = PDP.getFaults();

    if (faults.Brownout) {
      warn("The PDP is having a brownout");
    }

    if (faults.CanWarning) {
      warn("The PDP is having CAN issues");
    }

    if (faults.HardwareFault) {
      warn("The PDP is having hardware issues");
    }

    Boolean[] channelFaults = {
        faults.Channel0BreakerFault,
        faults.Channel1BreakerFault,
        faults.Channel2BreakerFault,
        faults.Channel3BreakerFault,
        faults.Channel4BreakerFault,
        faults.Channel5BreakerFault,
        faults.Channel6BreakerFault,
        faults.Channel7BreakerFault,
        faults.Channel8BreakerFault,
        faults.Channel9BreakerFault,
        faults.Channel10BreakerFault,
        faults.Channel11BreakerFault,
        faults.Channel12BreakerFault,
        faults.Channel13BreakerFault,
        faults.Channel14BreakerFault,
        faults.Channel15BreakerFault,
        faults.Channel16BreakerFault,
        faults.Channel17BreakerFault,
        faults.Channel18BreakerFault,
        faults.Channel19BreakerFault,
        faults.Channel20BreakerFault,
        faults.Channel21BreakerFault,
        faults.Channel22BreakerFault,
        faults.Channel23BreakerFault
    };
    boolean hasBreakerFault = false;
    String breakerFaultMessage = "The PDP is having issues with breakers: ";
    for (int x = 0; x < channelFaults.length; x++) {
      if (channelFaults[x]) {
        breakerFaultMessage += (x + ", ");
        hasBreakerFault = true;
      }
    }
    if (hasBreakerFault) {
      // warn(breakerFaultMessage);
    }
  }

  public static void checkCANCoderFaults(CANCoder encoder) {
    CANCoderFaults faults = new CANCoderFaults();
    encoder.getFaults(faults);
    if (faults.APIError) {
      warn("CANCoder " + encoder.getDeviceID() + " is having API issues");
    }
    if (faults.HardwareFault) {
      warn("CANCoder " + encoder.getDeviceID() + " is having Hardware issues");
    }
    if (faults.MagnetTooWeak) {
      warn("CANCoder " + encoder.getDeviceID() + " has too weak of a Magnet");
    }
    if (faults.UnderVoltage) {
      warn("CANCoder " + encoder.getDeviceID() + " is having under voltage issues");
    }
  }

  public static void warn(String msg) {
    DriverStation.reportError(msg, true);
  }

  public enum MotorFault {
    kBrownout(0x01),
    kCanMismatch(0x02),
    kEepromCrc(0x04),
    kEepromRead(0x08),
    kHardLimitForward(0x10),
    kHardLimitReverse(0x20),
    kSoftLimitForward(0x40),
    kSoftLimitReverse(0x80),
    kStall(0x100),
    kSupplyVoltage(0x200);

    private final int value;

    MotorFault(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }
}
