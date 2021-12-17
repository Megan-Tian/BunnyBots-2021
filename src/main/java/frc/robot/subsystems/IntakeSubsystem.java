// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
 

  private static final int intakePort = 5;
  private WPI_TalonSRX intakeMotor;

  

  /** Creates a new ActuatorSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new WPI_TalonSRX(intakePort);

      // need configSelectedFeedbackSensor line to make graph show up in Phoenix Tuner
    // use to dial in other PID values


    
  }

  public void SpinIntake(double speed){
    // System.out.println("SpinIntake");
    //speed = RobotContainer.leftJoystick.getZ();
    intakeMotor.set(ControlMode.PercentOutput, speed);
    //System.out.println("Speed : "+speed);
  }



  /*
  
  public void actuateIntake(double targetPosition){
    //System.out.println("LiftIntake");
    double motorCurrent = actuatorMotor.getStatorCurrent();
    
    if (Math.abs(motorCurrent) > motorCurrentLimit) {
      System.out.println("actuator motor output current:" + motorCurrent +". Exceded. Push no further");
      if (motorCurrent < 0) {
        targetPosition++;
      } else {
        targetPosition--; // because encoders weren't working 
      }
    }
    //targetPosition = 4096/3;
    //RobotContainer.leftJoystick.get
    //double encoderTargetPosition =  targetPosition = (targetPosition + 1)*1024;
    
    double encoderTargetPosition =  targetPosition = targetPosition *1024;

    // if (encoderTargetPosition > actuatorEncoderTickRotationLimit) {
    //   //Don't rotate beyond this point.
    //   encoderTargetPosition = actuatorEncoderTickRotationLimit;
    // }

    if (encoderTargetPosition != currentTargetPosition){

      double currentPosition  = actuatorMotor.getSelectedSensorPosition();
      
     // double currentPosition  = actuatorMotor.getSensorCollection().getQuadraturePosition();
      System.out.println("currentPosition/targetPosition:" +  "\t" + currentPosition + "\t" + currentTargetPosition); 
      currentTargetPosition = encoderTargetPosition;
    } 
    
    int kMeasuredPosHorizontal = 1023; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = actuatorMotor.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

     double maxGravityFF = 0.5; // to be set experimentally TODO
    
     //actuatorMotor.set(ControlMode.PercentOutput, targetPosition );
    //actuatorMotor.set(ControlMode.MotionMagic, currentTargetPosition);
    // actuatorMotor.set(TalonSRXControlMode.MotionMagic, currentTargetPosition);
    actuatorMotor.set(ControlMode.MotionMagic, -currentTargetPosition, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar); 
  }

*/

  
  @Override
  public void periodic() {
  }
}
