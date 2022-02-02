// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Arm.commands.ArmToPosition;
import frc.robot.Arm.commands.ArmToPositionIntake;
import frc.robot.Arm.commands.ArmToPositionShoot;
import frc.robot.Arm.subsystems.Arm;
import frc.robot.Arm.subsystems.Arm.ArmState;
import frc.robot.Subsystems.Pigeon;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Arm m_arm;
  private Pigeon m_pigeon;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_pigeon = new Pigeon();
    DoubleSupplier xAcc = () -> m_pigeon.getAcceleration()[0];
    DoubleSupplier zAcc = () -> m_pigeon.getAcceleration()[2];
    m_arm = new Arm(xAcc,zAcc);
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
    return new ArmToPositionIntake(m_arm);
  }
  public Command getArmToPostionCommandShoot() {
    return new ArmToPositionShoot(m_arm);
  }
  public Command testPos() {
    return new InstantCommand(()->m_arm.testPos());
  }
}