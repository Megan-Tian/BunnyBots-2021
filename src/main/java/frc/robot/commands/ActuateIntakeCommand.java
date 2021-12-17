// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ActuateIntakeCommand extends CommandBase {
  private ActuatorSubsystem m_actuatorSubsytem;
  private double m_position;

  /** Creates a new ActuateIntakeCommand. */
  public ActuateIntakeCommand(ActuatorSubsystem actuator, double targetPosition) {
    m_position = targetPosition;
    m_actuatorSubsytem = actuator;
    addRequirements(actuator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ActuateIntakeCommand intialize(): " + m_position);
    // m_intakeSubsystem.setZero();

    m_actuatorSubsytem.setActuatorTargetPosition(m_position);
    // if (m_lower) {
    //   m_intakeSubsystem.setActuatorTargetPosition(0);
    //   //lowers intake subsystem to slimyy wet concreteo fo bunny bot field
    // }
    // else{
    //   m_intakeSubsystem.setActuatorTargetPosition(300);
    //   System.out.println("Raise intake");
    // // set target position
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("ActuateIntakeCommand execute: " + m_position);
    //m_actuatorSubsytem.actuateIntake2(m_position);

    // m_intakeSubsystem.setActuatorTargetPosition(300);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
    // boolean isActuationFinished = m_intakeSubsystem.isActuationFinished(m_lower);
    // return isActuationFinished;
  }
}
