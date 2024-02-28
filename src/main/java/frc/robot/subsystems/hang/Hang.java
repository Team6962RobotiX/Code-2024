// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hang;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.subsystems.drive.SwerveDrive;

public class Hang extends SubsystemBase {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private State state = State.OFF; 
    
    private AHRS gyro; 
    private double gyroRotation; 
    
    public static enum State {
        EXTEND,
        RETRACT,
        OFF
    }
      

  /** Creates a new ExampleSubsystem. */
  public Hang(SwerveDrive swerve) {
    gyro = swerve.getGyro();
    // Gyro = SwerveDrive.getGyro()

    leftMotor = new CANSparkMax(CAN.LEFT_MOTOR, MotorType.kBrushless); // TODO
    //todo: figure out which motor is inverted and confugire it as so
    SparkMaxUtil.configureAndLog(this, leftMotor, false, IdleMode.kBrake);
    SparkMaxUtil.save(leftMotor);

    rightMotor = new CANSparkMax(CAN.RIGHT_MOTOR, MotorType.kBrushless); // TODO
    
    SparkMaxUtil.configureAndLog(this, rightMotor, false, IdleMode.kBrake);
    SparkMaxUtil.save(rightMotor);   

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    SparkMaxUtil.configureEncoder(leftMotor, 1.0);
    SparkMaxUtil.configureEncoder(rightMotor, 1.0);
 
  }
  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }


  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;
    if (RobotState.isDisabled()) {
      state = State.OFF;
    }

    switch(state) {
      case OFF:
        leftMotor.set(0);
        rightMotor.set(0);
        break;
      case EXTEND:
        if(rightEncoder.getPosition() < Constants.HANG.EXTEND_MAX_ROTATIONS){
            rightMotor.set(Preferences.HANG.LEFT_MOTOR_EXTEND_POWER);
        }
        else{
            rightEncoder.setPosition(0);
            state = State.OFF; 
        }
        if(leftEncoder.getPosition() < Constants.HANG.EXTEND_MAX_ROTATIONS){
            leftMotor.set(Preferences.HANG.RIGHT_MOTOR_EXTEND_POWER);
        }
        else{
            leftEncoder.setPosition(0);
            state = State.OFF; 
        }

        break;
      case RETRACT:
        gyroRotation = gyro.getRoll();
        // if it's tilting to the left and it can still retract
        if(gyroRotation > Preferences.HANG.MAX_ROLL_ANGLE && leftEncoder.getPosition() > -Constants.HANG.EXTEND_MAX_ROTATIONS){
            // then retract the left side
            leftMotor.set(Preferences.HANG.LEFT_MOTOR_RETRACT_POWER);
        }
        // if it's tilting to the right and it can still retract
        else if(gyroRotation < -Preferences.HANG.MAX_ROLL_ANGLE && rightEncoder.getPosition() > -Constants.HANG.EXTEND_MAX_ROTATIONS){
            // then retract the right side
            rightMotor.set(Preferences.HANG.RIGHT_MOTOR_RETRACT_POWER);
        }
        else{
            // otherwise, retract on all arms than can still retract
            if (rightEncoder.getPosition() > -Constants.HANG.EXTEND_MAX_ROTATIONS){
                rightMotor.set(Preferences.HANG.RIGHT_MOTOR_RETRACT_POWER);
            }
            else{
                rightEncoder.setPosition(0);
            }
            if (leftEncoder.getPosition() > -Constants.HANG.EXTEND_MAX_ROTATIONS){
                leftMotor.set(Preferences.HANG.LEFT_MOTOR_RETRACT_POWER);
            }
            else{
                leftEncoder.setPosition(0);
            }
        }
        break;
    }

    leftEncoder.getPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
