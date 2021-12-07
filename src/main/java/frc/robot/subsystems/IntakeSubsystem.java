// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private static final int intakePort = 6;
  private static final int actuatorPort = 7;
  private TalonSRX intakeMotor;
  private TalonSRX actuatorMotor;
  /** Creates a new ActuatorSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonSRX(intakePort);
    actuatorMotor = new TalonSRX(actuatorPort);
  }
  public void SpinIntake(double speed){
    System.out.println("SpinIntake");
   //intakeMotor.set(ControlMode.PercentOutput, speed);
  }
  public void LiftIntake(){
    System.out.println("LiftIntake");
  }
  public void LowerIntake(){
    System.out.println("LowerIntake");
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
