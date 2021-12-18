// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private static final int frontRightMotorPort = 1;
  private static final int frontLeftMotorPort = 3;
  private static final int backRightMotorPort = 2;
  private static final int backLeftMotorPort = 4;
  public static final int FRONT_RIGHT_DRIVE_CHANNEL = 1;
	public static final int BACK_RIGHT_DRIVE_CHANNEL = 2;
	public static final int FRONT_LEFT_DRIVE_CHANNEL = 3;
	public static final int BACK_LEFT_DRIVE_CHANNEL = 4;


  private TalonSRX frontRightMotor;
  private TalonSRX frontLeftMotor;
  private TalonSRX backRightMotor;
  private TalonSRX backLeftMotor;

  public DriveSubsystem() {
    frontRightMotor = new TalonSRX(frontRightMotorPort);
    frontLeftMotor = new TalonSRX(frontLeftMotorPort);
    backRightMotor = new TalonSRX(backRightMotorPort);
    backLeftMotor = new TalonSRX(backLeftMotorPort);
    backRightMotor.follow(frontRightMotor);
    backLeftMotor.follow(frontLeftMotor);
    
  }

  @Override
  public void periodic() {
     // m_motor.set(TalonSRXControlMode.PercentOutput, m_joystick.getY());
   //m_motor.set( m_joystick.getY());

   frontRightMotor.set(ControlMode.PercentOutput, RobotContainer.rightJoystick.getY());
   frontLeftMotor.set(ControlMode.PercentOutput, -RobotContainer.leftJoystick.getY());
    // This method will be called once per scheduler ru n
  }
  
  public void drive(double x, double y){
  // frontRightMotor.set(ControlMode.PercentOutput, x);
  // frontLeftMotor.set(ControlMode.PercentOutput, y);
  }
}
