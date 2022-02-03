// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Arm.commands.ArmToPosition;
import frc.robot.Arm.subsystems.Arm;
import frc.robot.Arm.subsystems.Arm.ArmState;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Arm m_arm;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_arm = new Arm();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
  public Command getArmToPostionCommand(ArmState state) {
    return new ArmToPosition(m_arm, state);
  }
  public Command getArmToPostionCommand(double positionInDegrees) {
    return new ArmToPosition(m_arm, positionInDegrees);
  }
  public Command updateFromSDB() {
    return new InstantCommand(()->m_arm.updateFromSDB());
  }
  public Command getArmToPostionCommandIntake() {
    InstantCommand command = new InstantCommand(()->m_arm.setGoal(ArmState.INTAKE.getRotations()));
    command.setName("Intke");
    return command;
  }
  public Command getArmToPostionCommandShoot() {
    InstantCommand command = new InstantCommand(()->m_arm.setGoal(ArmState.SHOOT.getRotations()));
    command.setName("shot");
    return command;
  }
  public Command getArmToPostionCommand() {
    InstantCommand command = new InstantCommand(()->m_arm.setGoal(SmartDashboard.getNumber("go to angle", 0)));
    command.setName("angle");
    return command;
  }
  public Command testPos() {
    return new InstantCommand(()->m_arm.testPos());
  }
  public void updateSDB() {
    SmartDashboard.putNumber("get pos",m_arm.getPos());
    SmartDashboard.putNumber("get vel",m_arm.getVel());
  }
}