// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.humanIO.Joysticks;
import frc.robot.humanIO.PS5Controller.Button;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;

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
    private final Arm m_arm = new Arm();

    private final Joysticks m_Joysticks = new Joysticks();

    private boolean _fieldRelative = true;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        m_Drivetrain.setDefaultCommand(
                new RunCommand(
                        () -> m_Drivetrain.drive(
                                m_Joysticks.getDriveX() * SwerveModuleConstants.freeSpeedMetersPerSecond,
                                m_Joysticks.getDriveY() * SwerveModuleConstants.freeSpeedMetersPerSecond,
                                m_Joysticks.getSteerX() * 11.5,
                                _fieldRelative),
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
        m_Joysticks.getButton(Button.kShare)
                .whenPressed(() -> m_Drivetrain.resetYaw());

        m_Joysticks.getButton(Button.kOptions)
                .whenPressed(() -> _fieldRelative = !_fieldRelative); // toggle field relative mode

        m_Joysticks.getButton(Button.kTriangle).toggleWhenPressed(new StartEndCommand(
                () -> m_arm.getActiveGoalCommand(ArmConstants.shootAngle).schedule(),
                () -> m_arm.getActiveGoalCommand(ArmConstants.intakeAngle).schedule(),
                m_arm));
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
