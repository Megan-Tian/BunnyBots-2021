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

public class ActuatorSubsystem extends SubsystemBase {
  /** Creates a new ActuatorSubsystem. */
  private static final int actuatorPort = 7;
  private WPI_TalonSRX actuatorMotor;

   // ENCODER POSITIONS 
   public static final double INTAKE_LOW_POSITION = 400; // may be 0? TODO
   public static final double INTAKE_DEPOSIT_HIGH_POSITION = 2500; 
   public static final double INTAKE_INTO_TOTE_POSITION = 2800; 

   //When the actuator arm presses against something, the motor current will increase.
  //If the motor current exceeds "motorCurrentLimit", stop pushing the motor.
  private double motorCurrentLimit = 1; 

  private double actuatorEncoderTickRotationLimit = -100;

	public static final int ACTUATOR_VELOCITY = 200; 
	public static final int ACTUATOR_ACCELERATION = 150; 

  public static double MOTION_MAGIC_P = 10;
  public static double MOTION_MAGIC_I = 0;//0.005;
  
	public static double MOTION_MAGIC_D = 0;//0.05;
  //public static double MOTION_MAGIC_F = 0.0;
  public static double MOTION_MAGIC_F = 0.5 * 1023 / (0.5 * 25794);
  //public static double MOTION_MAGIC_F = 0.0;

  private double m_targetPosition = 0;

  public ActuatorSubsystem() {
    actuatorMotor = new WPI_TalonSRX(actuatorPort);
    actuatorMotor.configFactoryDefault(); 
    //actuatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    ErrorCode configResult = actuatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
   
    // need configSelectedFeedbackSensor line to make graph show up in Phoenix Tuner
    // use to dial in other PID values 
    // set factory default

    actuatorMotor.setSensorPhase(false);

    // TODO stop @ dialing kP 

    actuatorMotor.config_kP(0, MOTION_MAGIC_P, 0);
    actuatorMotor.config_kI(0, MOTION_MAGIC_I, 0);
    actuatorMotor.config_kD(0, MOTION_MAGIC_D, 0);
    actuatorMotor.config_kF(0, MOTION_MAGIC_F, 0);
  }

  @Override
  public void periodic() {
    actuateIntake2(m_targetPosition);
  }

  public void disableActuator() {
    actuatorMotor.set(ControlMode.PercentOutput, 0);
    System.out.println("Disabling actuator");
  }

  public void actuateIntake2(double targetPosition) {
    // IMPORTANT !!!!!!
    // POSITIONS ARE SET IN THE BUTTONS VIA THE SetActuatorTargetPosition METHOD

    double tolerance = 50; 
    double distanceFromTarget = Math.abs(actuatorMotor.getSelectedSensorPosition() - targetPosition); 
    if (targetPosition == 0 && distanceFromTarget <= tolerance) {
      disableActuator();
      return;
    }  
    
    double motorCurrent = actuatorMotor.getStatorCurrent();
    //double zPosition = (RobotContainer.leftJoystick.getZ() + 1) * 1024;
    
    if (Math.abs(motorCurrent) > motorCurrentLimit) {
      System.out.println("actuator motor output current:" + motorCurrent +". Exceded. Push no further");
      disableActuator();
      // if (motorCurrent < 0) {
      //   m_targetPosition++;
      // } else {
      //   m_targetPosition--; // because encoders weren't working 
      // }
    }

    

    int kMeasuredPosHorizontal = 127; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = actuatorMotor.getSelectedSensorPosition();
    // System.out.println("currentPos/target position" + currentPos + "\t" + m_targetPosition);
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    double maxGravityFF = 0.5; // experimentally determined to be between 0.25
    double demand2 = maxGravityFF * cosineScalar;
    // System.out.println("Demand 2 : " + demand2);
    actuatorMotor.set(ControlMode.MotionMagic, targetPosition);
 
    // actuatorMotor.set(ControlMode.MotionMagic, m_targetPosition, DemandType.ArbitraryFeedForward, demand2);
    System.out.println("End Position/Target Position :"+currentPos + "\t" + m_targetPosition);

    double delta = 50;
    // double position = frontLeftMotor.getSelectedSensorPosition();

      if(Math.abs(m_targetPosition - currentPos) < delta) {
     
       // frontLeftMotor.set(ControlMode.Position, newTargetPosition);
       System.out.println("End Position/Target Position :"+currentPos + "\t" + m_targetPosition);
       // System.out.println("getPosition :"+ encoderPosition);
      }
  }

  public void setZero (){
    actuatorMotor.setSelectedSensorPosition(0);
  }

  public boolean isActuationFinished(boolean isLowering) {
    
    double position = actuatorMotor.getSelectedSensorPosition();
  

    System.out.println("isActuationFinished:position:" + position);
   // actuatorMotor.configVoltageMeasurementFilter()

    // if (isLowering == true && position < -100) { // if the actuator is lowering and is almost done lowering
    //   return true;
    // } else if (!isLowering && position > 100) { // if the actuator is raising and is almost done raising
    //   return true; 
    // }

    return false; 
  }

  public void setActuatorTargetPosition(double targetPosition) {
    // actuatorMotor.set(ControlMode.MotionMagic, targetPosition);
    this.m_targetPosition = targetPosition; 

  }
}
