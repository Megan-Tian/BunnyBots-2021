// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class BunnyCollector extends SubsystemBase {
  /** Creates a new BunnyCollector. */
  private static final int bunnyCollectorPort = 5;
  private TalonSRX bunnyCollectorMotor;

  public BunnyCollector() {
    bunnyCollectorMotor = new TalonSRX(bunnyCollectorPort);

  
  }
  public void rotateBunnyCollector() {
    // bunnyCollectorMotor.set(ControlMode.PercentOutput, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
