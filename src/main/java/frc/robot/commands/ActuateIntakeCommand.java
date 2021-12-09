// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ActuateIntakeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private boolean m_lower;

  /** Creates a new ActuateIntakeCommand. */
  public ActuateIntakeCommand(IntakeSubsystem intake, boolean lower) {
    m_lower = lower;
    m_intakeSubsystem = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_lower) {
      m_intakeSubsystem.setActuatorTargetPosition(0);
      //lowers intake subsystem to slimyy wet concreteo fo bunny bot field
    }
    else{
      m_intakeSubsystem.setActuatorTargetPosition(100);
    // set target positiom
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_lower)
      m_intakeSubsystem.LowerIntake();
    else
      m_intakeSubsystem.LiftIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isActuationFinished = m_intakeSubsystem.isActuationFinished(m_lower);
    return isActuationFinished;
  }
}
