// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ActuatorSubsystem;

public class ActuateIntakeCommand2 extends WaitCommand {
  private ActuatorSubsystem m_actuatorSubsytem;
  private double m_targetPosition;
  private boolean m_isLowering;

  /** Creates a new ActuateIntakeCommand2. */
  public ActuateIntakeCommand2(ActuatorSubsystem actuator, double targetPosition, boolean isLowering) {
    super(15);
    m_targetPosition = targetPosition;
    m_actuatorSubsytem = actuator;
    m_isLowering = isLowering;
    addRequirements(actuator);

    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_isLowering) {
      m_actuatorSubsytem.rotateActuator(0.3, m_targetPosition);
    } else {
      m_actuatorSubsytem.rotateActuator(-0.3, m_targetPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_actuatorSubsytem.disableActuator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rotationFinished = m_actuatorSubsytem.isActuationFinished(m_targetPosition);
    if (rotationFinished){
      System.out.println("actuator rotationFinished");
    }
    else if  (super.isFinished()) {
      System.out.println("actuator rotation timed out");
      return true;
    }
    
    return rotationFinished;
  }
}
