// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ActuatorSubsystem extends SubsystemBase {
  /** Creates a new ActuatorSubsystem. */
  private static final int actuatorPort = 7; // changed from port 7 to 6
  private WPI_TalonSRX actuatorMotor;

   // ENCODER POSITIONS 
   public static final double INTAKE_LOW_POSITION = 400; // may be 0? TODO
   public static final double INTAKE_DEPOSIT_HIGH_POSITION = 2000; //change back to 2000
   public static final double INTAKE_INTO_TOTE_POSITION = 2000; 

   //When the actuator arm presses against something, the motor current will increase.
  //If the motor current exceeds "motorCurrentLimit", stop pushing the motor.
  private double motorCurrentLimit = 3; 

  private double actuatorEncoderTickRotationLimit = -100;

	public static final int ACTUATOR_VELOCITY = 150; 
  public static final int ACTUATOR_ACCELERATION = 50; 


  public static double MOTION_MAGIC_P = 10;
  public static double MOTION_MAGIC_I = 0;//0.005;
  
	public static double MOTION_MAGIC_D = 0;//0.05;
  //public static double MOTION_MAGIC_F = 0.0;
  public static double MOTION_MAGIC_F = 0.13 * 1023 / (0.13 * 25794);
  //public static double MOTION_MAGIC_F = 0.0;

  private double m_targetPosition = 400;

  public ActuatorSubsystem() {
    actuatorMotor = new WPI_TalonSRX(actuatorPort);
    actuatorMotor.configFactoryDefault(); 

    ErrorCode configResult = actuatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    actuatorMotor.setSensorPhase(false); // Ritchie - changed from false to true
    actuatorMotor.selectProfileSlot(0, 0);
    //actuatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    //configureForPID();
  }

  public void configureForPID() {

    
    // need configSelectedFeedbackSensor line to make graph show up in Phoenix Tuner
    // use to dial in other PID values 
    // set factory default

       //actuatorMotor.setInverted(true);
    // TODO stop @ dialing kP 

    actuatorMotor.config_kP(0, MOTION_MAGIC_P, 0);
    actuatorMotor.config_kI(0, MOTION_MAGIC_I, 0);
    actuatorMotor.config_kD(0, MOTION_MAGIC_D, 0);
    actuatorMotor.config_kF(0, MOTION_MAGIC_F, 0);


    	
    actuatorMotor.configMotionAcceleration(ACTUATOR_ACCELERATION, 0);
    	
    actuatorMotor.configMotionCruiseVelocity(ACTUATOR_VELOCITY, 0);
  }

  @Override
  public void periodic() {
    //actuateIntake2();

    // double zValue = RobotContainer.leftJoystick.getZ();
    // System.out.println(zValue);
    // zValue = (zValue + 1) * 1024;
    // actuateIntake2(zValue);
  }
  public void rotateActuator (double percentOutput, double targetPosition){
    actuatorMotor.set(ControlMode.PercentOutput, percentOutput);
    // System.out.println("Target Position/Current Position: " + targetPosition + "/" + actuatorMotor.getSelectedSensorPosition());
  }
  public void disableActuator() {
    actuatorMotor.set(ControlMode.PercentOutput, 0);
    actuatorMotor.stopMotor();
    System.out.println("Disabling Actuator, targetPosition, actualPosition: " + m_targetPosition + "\t" + actuatorMotor.getSelectedSensorPosition());
  }

  public void actuateIntake2() {
    // IMPORTANT !!!!!!
    // POSITIONS ARE SET IN THE BUTTONS VIA THE SetActuatorTargetPosition METHOD
    // System.out.println("targetPosition, actualPosition: " + m_targetPosition + "\t" + actuatorMotor.getSelectedSensorPosition());

    double tolerance = 50; 
    double distanceFromTarget = Math.abs(actuatorMotor.getSelectedSensorPosition() - m_targetPosition); 
    if (m_targetPosition == 0 && distanceFromTarget <= tolerance) {
       disableActuator();
      return;
    }  


    double motorCurrent = actuatorMotor.getStatorCurrent();
    //double zPosition = (RobotContainer.leftJoystick.getZ() + 1) * 1024;
    
    // if (Math.abs(motorCurrent) > motorCurrentLimit) {
    //   System.out.println("actuator motor output current:" + motorCurrent +". Exceded. Push no further");
    //   disableActuator();
    //   // if (motorCurrent < 0) {
    //   //   m_targetPosition++;
    //   // } else {
    //   //   m_targetPosition--; // because encoders weren't working 
    //   // }
    // }    

    int kMeasuredPosHorizontal = 400; //Position measured when arm is horizontal, originally 127
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    
    double currentPos = actuatorMotor.getSelectedSensorPosition();


    if (currentPos > 1000) {
      System.out.println("currentPosition > 1000; abort");
    }



    // System.out.println("currentPos/target position" + currentPos + "\t" + m_targetPosition);
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    double maxGravityFF = 0.13; // experimentally determined to be between 0.25-0.5 with old bin motor
    // 0.13 with large sim motor

    double demand2 = maxGravityFF * cosineScalar;
    // System.out.println("Demand 2 : " + demand2);
     //actuatorMotor.set(ControlMode.MotionMagic, m_targetPosition);
 
    //  actuatorMotor.set(ControlMode.MotionMagic, m_targetPosition, DemandType.ArbitraryFeedForward, demand2);
    System.out.println("Position/Target Position/degrees/cosineScalar:"+currentPos + "\t" + m_targetPosition + "\t" + degrees + "\t" + cosineScalar) ;
   
    double delta = 50;
    // double position = frontLeftMotor.getSelectedSensorPosition();

      if(Math.abs(m_targetPosition - currentPos) < delta) {
      disableActuator();
       // frontLeftMotor.set(ControlMode.Position, newTargetPosition);
       System.out.println("End Position/Target Position :"+currentPos + "\t" + m_targetPosition);
       // System.out.println("getPosition :"+ encoderPosition);
      }
  }

  public void setZero (){
    actuatorMotor.setSelectedSensorPosition(0);
  }

  public boolean isActuationFinished(double targetPosition) {
    
    double position = actuatorMotor.getSelectedSensorPosition();
    double delta = 50;
    if (Math.abs(position-targetPosition)<delta) {
      return true;
    }
    
    return false;
   // actuatorMotor.configVoltageMeasurementFilter()

    // if (isLowering == true && position < -100) { // if the actuator is lowering and is almost done lowering
    //   return true;
    // } else if (!isLowering && position > 100) { // if the actuator is raising and is almost done raising
    //   return true; 
    // }


  }

  public void setActuatorTargetPosition(double targetPosition) {
    // actuatorMotor.set(ControlMode.MotionMagic, targetPosition);
    this.m_targetPosition = targetPosition; 

  }
}
