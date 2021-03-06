// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import  frc.robot.subsystems.BunnyCollector;

public class BunnyPickupCommand extends CommandBase {
  
private BunnyCollector bunnyCollectorSubsystem;
private double m_targetPosition;

/** Creates a new bunnypickupcommand. */
  public BunnyPickupCommand(BunnyCollector bunnyCollectorSubsystem, double position) {
    this.bunnyCollectorSubsystem = bunnyCollectorSubsystem;
    m_targetPosition = position;
    bunnyCollectorSubsystem.setTargetPosition(position);
	// Use addRequirements() here to declare subsystem dependencies.
    addRequirements(bunnyCollectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.bunnyCollectorSubsystem.rotateBunnyCollector(m_targetPosition);
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
