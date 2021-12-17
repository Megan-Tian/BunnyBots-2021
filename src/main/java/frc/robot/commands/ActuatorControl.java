// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ActuatorControl extends CommandBase {
  /** Creates a new ActuatorControl. */
  private ActuatorSubsystem m_actuatorSubsystem;

  public ActuatorControl(ActuatorSubsystem actuator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_actuatorSubsystem = actuator;
    addRequirements(actuator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_intakesubsystem.setZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // m_actuatorSubsystem.actuateIntake2(0);
    //m_intakesubsystem.actuateIntake(RobotContainer.leftJoystick.getZ()); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
