// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Arm m_arm;
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
  private void configureButtonBindings() {
    // m_joysticks.getButton(Button.kA).whenHeld(new SetPrecent(m_arm, 0.1)); //SmartDashboard.getNumber("precent", 0)));
  }

  public double getPrecent(){
    return this.m_arm.getPrecent();
  }

  public double getVoltageCompensationNominalVoltage(){
    return this.m_arm.getCurrentVoltage();
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

  public void getSetPrecent(){
    double precent = SmartDashboard.getNumber("precent", 0)
    * Math.cos(Math.toRadians(SmartDashboard.getNumber("real angle", 0)))
    + SmartDashboard.getNumber("static", 0);
    SmartDashboard.putNumber("cos", Math.cos(Math.toRadians(SmartDashboard.getNumber("real angle", 0))));
    SmartDashboard.putNumber("input precent", precent);
    this.m_arm.setPrecent(-precent);
  }

  public double getRealAngle(){
    return this.m_arm.getAngle();
  }
}
