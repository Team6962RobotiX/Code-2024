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
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  CANSparkMax output1;
  CANSparkMax output2;


  @Override
  public void robotInit() {

    //Joystick
    joystick0 = new Joystick(0);

    //Sparks
    rbank1 = new CANSparkMax(9,MotorType.kBrushless);
    lbank1 = new CANSparkMax(12,MotorType.kBrushless); 
    lbank2 = new CANSparkMax(13,MotorType.kBrushless); 
    rbank2 = new CANSparkMax(14,MotorType.kBrushless);    
    rbank2.follow(rbank1);
    lbank2.follow(lbank1);
    leftEncoder = lbank1.getEncoder();
    rightEncoder = rbank1.getEncoder();

    //DifferentialDrive myDrive = new DifferentialDrive(lbank1,rbank1);
  
    output1 = new CANSparkMax(11,MotorType.kBrushless);
    output2 = new CANSparkMax(10,MotorType.kBrushless);
  }

    
  @Override
  public void robotPeriodic() {


  }

  @Override
  public void autonomousInit() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  
  @Override
  public void autonomousPeriodic() {
 
    System.out.println("left encoder");
    System.out.println(leftEncoder.getPosition());
    System.out.println("right encoder");
    System.out.println(rightEncoder.getPosition());

    if (leftEncoder.getPosition() <= 35 ) {
      System.out.println("In ");
       lbank1.set(0.1);
       rbank1.set(-0.1);
    } 
    
  
    

 
System.out.println("left encoder");
System.out.println(leftEncoder.getPosition());
System.out.println("right encoder");
System.out.println(rightEncoder.getPosition());

if (leftEncoder.getPosition() <= 50 && leftEncoder.getPosition() >35) {
  System.out.println("In ");
   lbank1.set(0.1);
   rbank1.set(0.1);


   
}


if (leftEncoder.getPosition() <= 82 && leftEncoder.getPosition() > 50) {
  System.out.println("In ");
   lbank1.set(0.1);
   rbank1.set(-0.1);
} 

if (leftEncoder.getPosition() > 82) {
  System.out.println("In ");
   lbank1.set(0);
   rbank1.set(0);
  output1.set(0.3);
  output2.set(0.3);
  }
  }
  @Override
  public void teleopInit() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    output1.set(0);
    output2.set(0);
  }

  @Override
  public void teleopPeriodic() {
    

    double limitTurnSpeed = 0.3;
    double limitSpeed = 0.7; 
    
    double joystickLValue = 
        (-joystick0.getRawAxis(1) * limitSpeed + (joystick0.getRawAxis(2) * limitTurnSpeed));
    double joystickRValue =
        (-joystick0.getRawAxis(1) * limitSpeed - (joystick0.getRawAxis(2) * limitTurnSpeed));
    System.out.println("left encoder");
    System.out.println(leftEncoder.getPosition());
    System.out.println("right encoder");
    System.out.println(rightEncoder.getPosition());

    rbank1.set(-joystickRValue);
    lbank1.set(joystickLValue);
    
    if (joystick0.getRawButton(1)) {
      output1.set(0.3);
      output2.set(0.3);
    
    }
    else if (joystick0.getRawButton(2)){
      output2.set(1);
    }
    else if (joystick0.getRawButton(3)){
      
      output1.set(1);
}
    else if (joystick0.getRawButton(4)){
      output1.set(0);
      output2.set(0);
    }
    else if (joystick0.getRawButton(6)){
      output1.set(-0.3);
      output2.set(-0.3);
    } 

    //Amartya is him
  }


}

