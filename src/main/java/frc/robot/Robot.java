package frc.robot;


import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  PWMSparkMax lbank;
  PWMSparkMax rbank;
  DifferentialDrive myDrive;
  
  @Override
  public void robotInit() {

    //Joystick
    joystick0 = new Joystick(0);

    //Sparks
    lbank = new PWMSparkMax(0); 
    rbank = new PWMSparkMax(1); 

    //TankDrive thingy dink
    myDrive = new DifferentialDrive(lbank,rbank);
  
  }
    
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    System.out.print("hi");
  }

  @Override
  public void teleopPeriodic() {

    lbank.setVoltage(12);
    rbank.setVoltage(12);

    double limitTurnSpeed = 0.75; 

    double joystickLValue = 
        (-joystick0.getRawAxis(1) + (joystick0.getRawAxis(2) * limitTurnSpeed));
    double joystickRValue =
        (-joystick0.getRawAxis(1) - (joystick0.getRawAxis(2) * limitTurnSpeed));

    myDrive.tankDrive(-joystickLValue, -joystickRValue,false);
  }

}

