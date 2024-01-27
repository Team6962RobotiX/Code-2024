package frc.robot;


import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;
import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANSparkMax;
import com.fasterxml.jackson.databind.util.RawValue;
import com.revrobotics.*;





import org.opencv.core.*;
import org.opencv.core.Mat;

import java.util.*;
public class Robot extends TimedRobot {
  Joystick joystick0;
  PWMSparkMax lbank1;
  PWMSparkMax lbank2;
  PWMSparkMax rbank1;
  PWMSparkMax rbank2;
  DifferentialDrive myDrive;
  
  @Override
  public void robotInit() {

    //Joystick
    joystick0 = new Joystick(0);

    //Sparks
    lbank1 = new PWMSparkMax(0);
    lbank2 = new PWMSparkMax(1); 
    rbank1 = new PWMSparkMax(2); 
    rbank2 = new PWMSparkMax(3);    
    DifferentialDrive myDrive = new DifferentialDrive(
      (double output) -> {
        lbank1.set(output);
        lbank2.set(output);
      },
      (double output) -> {
        rbank1.set(output);
        rbank2.set(output);
      });
  
    
  }

    
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    lbank1.setVoltage(12);
    lbank2.setVoltage(12);
    rbank1.setVoltage(12);
    rbank2.setVoltage(12);

    myDrive.tankDrive(-0.2, 0.2, false);
  }

  @Override
  public void teleopInit() {
    System.out.print("hi");
  }

  @Override
  public void teleopPeriodic() {

    lbank1.setVoltage(12);
    lbank2.setVoltage(12);
    rbank1.setVoltage(12);
    rbank2.setVoltage(12);

    double limitTurnSpeed = 0.5;
    double limitSpeed = 0.5; 

    double joystickLValue = 
        (-joystick0.getRawAxis(1) * limitSpeed + (joystick0.getRawAxis(2) * limitTurnSpeed));
    double joystickRValue =
        (-joystick0.getRawAxis(1) * limitSpeed - (joystick0.getRawAxis(2) * limitTurnSpeed));

    myDrive.tankDrive(-joystickLValue, joystickRValue,false);
  }


}

