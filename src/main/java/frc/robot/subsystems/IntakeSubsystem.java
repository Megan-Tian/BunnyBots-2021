// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private static final int intakePort = 5;
  private static final int actuatorPort = 7;
  private TalonSRX intakeMotor;
  private TalonSRX actuatorMotor;
  private double voltageLimit = 10;

  /** Creates a new ActuatorSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonSRX(intakePort);
    actuatorMotor = new TalonSRX(actuatorPort);
  }
  public void SpinIntake(double speed){
    // System.out.println("SpinIntake");
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }
  public void actuateIntake(double targetPosition){
    System.out.println("LiftIntake");

    //targetPosition = 4096/3;

    int kMeasuredPosHorizontal = 1023; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = actuatorMotor.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    double maxGravityFF = 0.1; // to be set experimentally TODO

    //frontLeftMotor.set(ControlMode.MotionMagic, targetPosition);
    actuatorMotor.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar); 
  }
  public void LowerIntake(){
    System.out.println("LowerIntake");
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // :)

  public boolean isActuationFinished(boolean isLowering) {
    
    double position = actuatorMotor.getSelectedSensorPosition();
    double outputVoltage = actuatorMotor.getMotorOutputVoltage();
   // actuatorMotor.configVoltageMeasurementFilter()
    if(outputVoltage > voltageLimit){
      System.out.println("Voltage Limit exceeded");
      return true;
    }
    System.out.println("Voltage Output : "+outputVoltage);
    System.out.println("Actuator Position: " + position);
    // TODO put in code from MotorTest confirming degrees of rotation

    //actuatorMotor.setSelectedSensorPosition(100);

    if (isLowering == true && position < 10) { // if the actuator is lowering and is almost done lowering
      // 10 is placeholder lol
      return true;
    } else if (!isLowering && position > 100) { // if the actuator is raising and is almost done raising
      // 100 also placeholder 
      // Angle of rotation ~180 so 100 is conservative estimate
      return true; 
    }

    return false; 
  }

  public void setActuatorTargetPosition(double targetPosition) {
    actuatorMotor.setSelectedSensorPosition(targetPosition);

  }


}
