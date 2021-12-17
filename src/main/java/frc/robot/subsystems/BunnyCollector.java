// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class BunnyCollector extends SubsystemBase {
  /** Creates a new BunnyCollector. */
  private static final int bunnyCollectorPort = 6;
  private WPI_TalonSRX bunnyCollectorMotor;
  private double m_targetPosition;
  private double motorCurrentLimit = 1; 

  public BunnyCollector() {
    bunnyCollectorMotor = new WPI_TalonSRX(bunnyCollectorPort);
    
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setTargetPosition(double target){
    m_targetPosition = target;
  }
  public void rotateBunnyCollector(double targetPosition) {
    // IMPORTANT !|!!!!
    // POSITIONS ARE SET IN THE BUTTONS VIA THE SetActuatorTargetPosition METHOD

    double motorCurrent = bunnyCollectorMotor.getStatorCurrent();
    //double zPosition = (RobotContainer.leftJoystick.getZ() + 1) * 1024;
 
    if (Math.abs(motorCurrent) > motorCurrentLimit) {
      System.out.println("actuator motor output current;" + motorCurrent +". Exceded, Push no further");
      if (motorCurrent < 0) {
        m_targetPosition++;
      } else {
        m_targetPosition--; // because encoders werent working 
      }
    }

    

    // System.out.println("Demand 2 : " + demand2);
    //frontLeftMotor.set(ControlMode.MotionMagic, targetPosition);
 
    bunnyCollectorMotor.set(ControlMode.MotionMagic, targetPosition);

    
  }
}
