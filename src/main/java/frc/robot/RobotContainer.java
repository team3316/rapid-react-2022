// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.autonomous.AutoSelector;
import frc.robot.commands.Shoot;
import frc.robot.humanIO.Joysticks;
import frc.robot.humanIO.PS5Controller.Button;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Trigger.Side;
import frc.robot.subsystems.Trigger.TriggerState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Drivetrain m_Drivetrain = new Drivetrain();

    private final Manipulator m_manipulator = new Manipulator();

    private final Joysticks m_Joysticks = new Joysticks();

    private final Trigger m_trigger = new Trigger();

    private final Arm m_arm = new Arm();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        AutoSelector.initialize(m_Drivetrain);

        // Configure the button bindings
        configureButtonBindings();
        m_Drivetrain.setDefaultCommand(
                new RunCommand(
                        () -> m_Drivetrain.drive(
                                m_Joysticks.getDriveX() * SwerveModuleConstants.freeSpeedMetersPerSecond,
                                m_Joysticks.getDriveY() * SwerveModuleConstants.freeSpeedMetersPerSecond,
                                m_Joysticks.getSteerX() * 11.5,
                                true),
                        m_Drivetrain));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        m_Joysticks.getButton(Button.kShare).whenPressed(() -> m_Drivetrain.resetYaw());

        m_Joysticks.getButton(Button.kL1)
        .toggleWhenPressed(
          new StartEndCommand(
            () -> m_manipulator.setState(ManipulatorState.COLLECT),
            () -> m_manipulator.setState(ManipulatorState.OFF), 
            m_manipulator)
          .withInterrupt(() -> m_manipulator.getCargoState().hasBoth()));
      
      m_Joysticks.getButton(Button.kR1)
        .whileHeld(new Shoot(m_manipulator));
  
      m_Joysticks.getButton(Button.kCross).whenHeld(this.getSetTriggerState(Side.LEFT));
      m_Joysticks.getButton(Button.kCircle).whenHeld(this.getSetTriggerState(Side.RIGHT));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return AutoSelector.get("Test Auto");
    }

    public Command getSetTriggerState(Side side){
      // TODO check if needed to end the command in another way
      return new StartEndCommand(() -> m_trigger.setState(TriggerState.OUT, side), () -> m_trigger.setState(TriggerState.IN, side));
    }

    public double getPrecent(){
      return this.m_arm.getPrecent();
    }
  
    public double getVoltageCompensationNominalVoltage(){
      return this.m_arm.getCurrentVoltage();
    }
    
  public void getSetPrecent(){
    double precent = SmartDashboard.getNumber("precent", 0)
    * Math.cos(Math.toRadians(SmartDashboard.getNumber("angle", 0)))
    + SmartDashboard.getNumber("static", 0);
    SmartDashboard.putNumber("input precent", precent);
    this.m_arm.setPrecent(-precent);
  }

  public double getRealAngle(){
    return this.m_arm.getAngle();
  }
  
}
