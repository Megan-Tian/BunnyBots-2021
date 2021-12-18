// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ActuateIntakeCommand;
import frc.robot.commands.ActuateIntakeCommand2;
import frc.robot.commands.ActuatorControl;
import frc.robot.commands.BunnyPickupCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ActuatorSubsystem;
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
  // public final BunnyCollector m_bunnyCollector = new BunnyCollector();
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ActuatorSubsystem m_actuatorSubsystem = new ActuatorSubsystem();

  // COMMANDS
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  // private final BunnyPickupCommand m_bunnyPickupCommand = new BunnyPickupCommand(m_bunnyCollector, -1000);
  public final DriveCommand m_driveCommand = new DriveCommand(m_driveSubsystem); 

  // !!! CHANGE LINE BELOW TO CHANGE INTAKE ROTATION SPEED - closer to 1 to increase speed, closer to 0 to decrease
  public final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem, 0.24); // started out as 0.18/0.19, adjust by 0.1 or 0.2 as needed
 
  public final IntakeCommand m_unloadIntakeCommand = new IntakeCommand(m_intakeSubsystem, -0.6);
  public final IntakeCommand m_reverseIntakeCommand = new IntakeCommand(m_intakeSubsystem, 0.6);
  public final ActuateIntakeCommand2 m_raiseIntake = new ActuateIntakeCommand2(m_actuatorSubsystem, ActuatorSubsystem.INTAKE_DEPOSIT_HIGH_POSITION, false);
  public final ActuateIntakeCommand2 m_lowerIntake = new ActuateIntakeCommand2(m_actuatorSubsystem, ActuatorSubsystem.INTAKE_LOW_POSITION, true);
  // public final ActuateIntakeCommand2 m_lowerIntakeSmall = new ActuateIntakeCommand2(m_actuatorSubsystem, ActuatorSubsystem.INTAKE_START, true);
  //public final ActuatorControl m_actuatorControl = new ActuatorControl(m_actuatorSubsystem);
 // public final ActuateIntakeCommand m_raiseIntoNeutralIntake = new ActuateIntakeCommand(m_actuatorSubsystem, ActuatorSubsystem.INTAKE_LOW_POSITION);


  // JOYSTICKS
  public static Joystick rightJoystick = new Joystick(0);
  public static Joystick leftJoystick = new Joystick(1);

  // BUTTONS
  // public JoystickButton bunnyCollectorButton = new JoystickButton(rightJoystick, 2);
  public JoystickButton intakeButton = new JoystickButton(rightJoystick, 1);
  public JoystickButton raiseintakeButton = new JoystickButton(rightJoystick, 2);
  public JoystickButton lowerintakeButton = new JoystickButton(leftJoystick, 2);
  public JoystickButton unloadIntakeButton = new JoystickButton(rightJoystick, 4); // spin in SAME DIRECTION as intakeButton, just faster
  public JoystickButton reverseIntakeButton = new JoystickButton(leftJoystick, 1); // spins intake in opposite direction to expel balls. Uses intakeCommand
  //public JoystickButton raiseIntakeToNeutralPositionButton = new JoystickButton(rightJoystick, 3);

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
    // bunnyCollectorButton.whileHeld(m_bunnyPickupCommand);
    intakeButton.whenHeld(m_intakeCommand);
    reverseIntakeButton.whenHeld(m_reverseIntakeCommand);
    unloadIntakeButton.whenHeld(m_unloadIntakeCommand);
    raiseintakeButton.whenPressed(m_raiseIntake);
    lowerintakeButton.whenPressed(m_lowerIntake);

    //raiseIntakeToNeutralPositionButton.whenPressed(m_raiseIntoNeutralIntake);

  }
  // public void startActuator(){
  //   m_actuatorControl.start();
  // }

  // public void stopActuator(){
  //   m_actuatorControl.cancel();
  // }

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
