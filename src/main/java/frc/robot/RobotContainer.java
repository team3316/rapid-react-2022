// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.autonomous.FollowTrajectory;
import frc.robot.commandGroups.ShootCollectShoot;
import frc.robot.commandGroups.ShootCollectTwoShoot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.OpenLeftTrigger;
import frc.robot.commands.OpenRightTrigger;
import frc.robot.humanIO.Joysticks;
import frc.robot.humanIO.PS5Controller.Button;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystems.trigger.Trigger;

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
    private final Manipulator m_Manipulator = new Manipulator();

    private final Trigger m_Trigger = new Trigger();

    private final Drivetrain m_Drivetrain = new Drivetrain();

    private final Arm m_arm = new Arm();

    private final Climber m_Climber = new Climber();

    private final Joysticks m_Joysticks = new Joysticks();

    private final FollowTrajectory m_followTrajectory = new FollowTrajectory(m_Drivetrain, "collect");
    private final FollowTrajectory m_followTrajectoryReversed = new FollowTrajectory(m_Drivetrain, "collect_rev");

    private final FollowTrajectory path1 = new FollowTrajectory(m_Drivetrain, "collect_two_path1");
    private final FollowTrajectory path2 = new FollowTrajectory(m_Drivetrain, "collect_two_path2");
    private final FollowTrajectory path3 = new FollowTrajectory(m_Drivetrain, "collect_two_path3");

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

        m_Climber.setDefaultCommand(
                new RunCommand(
                        () -> m_Climber.set(m_Joysticks.getClimbY()),
                        m_Climber));
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
        m_Joysticks.getOperatorButton(Button.kL1)
                .toggleWhenPressed(
                        new StartEndCommand(
                                () -> m_Manipulator.setState(ManipulatorState.COLLECT),
                                () -> m_Manipulator.setState(ManipulatorState.OFF),
                                m_Manipulator)
                                        .withInterrupt(() -> m_Manipulator.getCargoState().hasBoth()));

        m_Joysticks.getOperatorButton(Button.kSquare)
                .whileHeld(
                        new StartEndCommand(
                                () -> m_Manipulator.setState(ManipulatorState.SHOOT),
                                () -> m_Manipulator.setState(ManipulatorState.OFF),
                                m_Manipulator));

        m_Joysticks.getOperatorButton(Button.kR1)
                .whenPressed(
                        new ConditionalCommand(
                                new AutoShoot(m_Manipulator, m_Trigger),
                                new InstantCommand(),
                                () -> !m_arm.isLastGoalIntake())); // Don't shoot during intake

        m_Joysticks.getOperatorButton(Button.kCross)
                .whenHeld(new OpenLeftTrigger(m_Trigger));
        m_Joysticks.getOperatorButton(Button.kCircle)
                .whenHeld(new OpenRightTrigger(m_Trigger));

        this.m_Joysticks.getOperatorPOVButton(270).whenHeld(
                new StartEndCommand(
                        () -> this.m_Climber.setL(-0.1),
                        () -> this.m_Climber.setL(0.0),
                        m_Climber));

        this.m_Joysticks.getOperatorPOVButton(90).whenHeld(
                new StartEndCommand(
                        () -> this.m_Climber.setR(-0.1),
                        () -> this.m_Climber.setR(0.0),
                        m_Climber));

        m_Joysticks.getDriveButton(Button.kShare)
                .whenPressed(() -> m_Drivetrain.resetYaw());

        m_Joysticks.getDriveButton(Button.kOptions)
                .whenPressed(() -> _fieldRelative = !_fieldRelative); // toggle field relative mode

        m_Joysticks.getOperatorButton(Button.kTriangle).whenPressed(
                new ConditionalCommand(
                        new InstantCommand(() -> m_arm.getActiveGoalCommand(ArmConstants.shootAngle).schedule()),
                        new InstantCommand(() -> m_arm.getActiveGoalCommand(ArmConstants.intakeAngle).schedule()),
                        m_arm::isLastGoalIntake)
                                .beforeStarting(new InstantCommand(
                                        () -> m_Manipulator.setState(ManipulatorState.OFF),
                                        m_Manipulator)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new ShootCollectTwoShoot(m_arm, m_Manipulator, m_Trigger, path1, path2, path3);
    }

    public void disableInit() {
        m_arm.disabledInit();
        m_Drivetrain.disabledInit();
        m_Manipulator.setState(ManipulatorState.OFF);
    }

    public void calibrateDrivetrainSteering() {
        m_Drivetrain.calibrateSteering();
    }
}
