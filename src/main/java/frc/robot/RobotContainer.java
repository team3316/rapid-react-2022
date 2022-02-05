// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.humanIO.Joysticks;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Manipulator m_manipulator = new Manipulator();

  private final Joysticks m_joysticks = new Joysticks();
  private final Trigger m_trigger = new Trigger();
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
    m_joysticks.getButton(Button.kLeftBumper)
      .toggleWhenPressed(
        new StartEndCommand(
          () -> m_manipulator.setState(ManipulatorState.COLLECT),
          () -> m_manipulator.setState(ManipulatorState.OFF), 
          m_manipulator)
        .withInterrupt(() -> m_manipulator.getCargoState().hasBoth()));
    
    m_joysticks.getButton(Button.kRightBumper)
      .toggleWhenPressed(
        new StartEndCommand(
          () -> m_manipulator.setState(ManipulatorState.SHOOT),
          () -> m_manipulator.setState(ManipulatorState.OFF),
          m_manipulator));

    m_joysticks.getButton(Button.kA)
      .whenHeld(
        new StartEndCommand(
          () -> this.m_trigger.setLeftAngle(Constants.Trigger.outAngle), 
          () -> this.m_trigger.setLeftAngle(Constants.Trigger.inAngel)));
    m_joysticks.getButton(Button.kB)
      .whenHeld(
        new StartEndCommand(
          () -> this.m_trigger.setRightAngle(Constants.Trigger.outAngle), 
          () -> this.m_trigger.setRightAngle(Constants.Trigger.inAngel)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
