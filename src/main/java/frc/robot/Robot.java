package frc.robot;


import edu.wpi.first.cameraserver.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;
import edu.wpi.first.wpilibj.TimedRobot;
import com.fasterxml.jackson.databind.util.RawValue;
import com.revrobotics.*;





import org.opencv.core.*;
import org.opencv.core.Mat;

import java.util.*;
public class Robot extends TimedRobot {
  Joystick joystick0;
  CANSparkMax lbank1;
  CANSparkMax lbank2;
  CANSparkMax rbank1;
  CANSparkMax rbank2;
  DifferentialDrive myDrive;
  
  @Override
  public void robotInit() {

    //Joystick
    joystick0 = new Joystick(0);

    //Sparks
    rbank1 = new CANSparkMax(1,MotorType.kBrushless);
    lbank1 = new CANSparkMax(2,MotorType.kBrushless); 
    lbank2 = new CANSparkMax(3,MotorType.kBrushless); 
    rbank2 = new CANSparkMax(4,MotorType.kBrushless);    
    rbank2.follow(rbank1);
    lbank2.follow(lbank1);

    DifferentialDrive myDrive = new DifferentialDrive(lbank1,rbank1);
  
    
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

//qwertyuiopasdfghjklzxcvbnmmnbvcxzlkjhgfdsapoiuytrewqqazwsxedcrfvtgbyhnujmikolpplokimjunhybgtvfrcdexswzaq