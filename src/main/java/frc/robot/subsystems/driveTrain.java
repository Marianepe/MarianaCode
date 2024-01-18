// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.Num;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.Navx;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSpeedController;

import org.ejml.data.ZMatrix;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class driveTrain extends SubsystemBase {
  /** Creates a new driveTrain2. */
  /*TecbotSpeedController m1;
  TecbotSpeedController m2;
  TecbotSpeedController m3;
  TecbotSpeedController m4; */
  
  private CANSparkMax motor1;
 private CANSparkMax motor2;
 private CANSparkMax motor3;
 private CANSparkMax motor4;

 /*  boolean leftSideBalanced = false, rightSideBalanced = false;
  


  RelativeEncoder driveTrainEncoderL1, driveTrainEncoderL2, driveTrainEncoderR1, driveTrainEncoderR2;

  DoubleSolenoid transmition;

  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro(kGyroPort);*/
  
 
// m1 & m2 left 

  public driveTrain() {
    //motor1 = new CANSparkMax(11 , MotorType.kBrushless);
   // motor2 = new CANSparkMax(51, MotorType.kBrushless);
   
 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void drive(double l_x, double l_y,double r_x,double r_y){
    Robot.getRobotContainer().getOI().getPilot().setOffset(RobotMap.OFFSET);
    //double rotatingspeed = l_x + r_x;´
    //double advSpeed = l_y + r_y;´
    SmartDashboard.putNumber("Variable",l_x);
    motor1.set(-l_y);
    motor2.set(l_y);

    
  //public void DifferentialDrive() {
   // motor1.set(-l_y);
  //  motor2.set(l_y);
//Comand return drive(l_x, l_y, r_x, r_y);
  }

    
   /*  m1.set(leftSpeed);
    m2.set(leftSpeed);
    m3.set(rightSpeed);
    m4.set(rightSpeed);*/
  } 


  

  

 /*  public void resetEncoderDt(){
    driveTrainEncoderL1.setPosition(0);
    driveTrainEncoderL2.setPosition(0);
    driveTrainEncoderR1.setPosition(0);
    driveTrainEncoderR2.setPosition(0);
  }  */

  /*public double getDriveTrainFeet(){
   return -1 * driveTrainEncoderL1.getPosition() * TecbotConstants.kDriveTick2Feet;
  }*/
  
  /*public double getDriveTrainFeetR(){
    return driveTrainEncoderR1.getPosition();
  }*/

  /*public void driveForward(){
   m1.set(-RobotMap.chassisSpeedL);
   m2.set(-RobotMap.chassisSpeedL); 
   m3.set(RobotMap.chassisSpeedR);
   m4.set(RobotMap.chassisSpeedR); 
  }*/


  
  

  /*public void stop(){
    m1.set(0);
    m2.set(0);
    m3.set(0);
    m4.set(0);
  }*/
  
 // public boolean hasElapsed (double sec) {
    //sec = 1.5;
//return t.get() >= sec;
 //}
 

 

  



 

 

 



