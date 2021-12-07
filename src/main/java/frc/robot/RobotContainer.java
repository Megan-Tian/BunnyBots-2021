// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ActuateIntakeCommand;
import frc.robot.commands.BunnyPickupCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.BunnyCollector;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final BunnyCollector m_bunnyCollector = new BunnyCollector();
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();


  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final BunnyPickupCommand m_bunnyPickupCommand = new BunnyPickupCommand(m_bunnyCollector);
  public final DriveCommand m_driveCommand = new DriveCommand(m_driveSubsystem); 
  public final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem, 1);
  public final ActuateIntakeCommand m_raiseIntake = new ActuateIntakeCommand(m_intakeSubsystem, false);
  public final ActuateIntakeCommand m_lowerIntake = new ActuateIntakeCommand(m_intakeSubsystem, true);
  public static Joystick leftJoystick = new Joystick(0);
    public static Joystick rightJoystick = new Joystick(1);
    public JoystickButton bunnyCollectorButton = new JoystickButton(rightJoystick, 2);
    public JoystickButton intakeButton = new JoystickButton(leftJoystick, 1);
    public JoystickButton raiseintakeButton = new JoystickButton(leftJoystick, 11);
    public JoystickButton lowerintakeButton = new JoystickButton(leftJoystick, 6);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    bunnyCollectorButton.whileHeld(m_bunnyPickupCommand);
    intakeButton.whileHeld(m_intakeCommand);
    raiseintakeButton.whenPressed(m_raiseIntake);
    lowerintakeButton.whenPressed(m_lowerIntake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
