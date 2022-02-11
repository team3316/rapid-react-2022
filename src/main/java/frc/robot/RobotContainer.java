// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.humanIO.Joysticks;
import frc.robot.humanIO.PS5Controller.Button;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystems.trigger.Trigger;
import frc.robot.subsystems.Climber;
=======
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.humanIO.Joysticks;
import frc.robot.humanIO.PS5Controller.Button;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;
>>>>>>> 0d31a10 (added joysticks control)

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
<<<<<<< HEAD
    // The robot's subsystems and commands are defined here...
    private final Manipulator m_Manipulator = new Manipulator();

    private final Trigger m_Trigger = new Trigger();

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
        m_Joysticks.getButton(Button.kL1)
                .toggleWhenPressed(
                        new StartEndCommand(
                                () -> m_Manipulator.setState(ManipulatorState.COLLECT),
                                () -> m_Manipulator.setState(ManipulatorState.OFF),
                                m_Manipulator)
                                        .withInterrupt(() -> m_Manipulator.getCargoState().hasBoth()));

        m_Joysticks.getButton(Button.kR1)
                .toggleWhenPressed(
                        new StartEndCommand(
                                () -> m_Manipulator.setState(ManipulatorState.SHOOT),
                                () -> m_Manipulator.setState(ManipulatorState.OFF),
                                m_Manipulator));

        m_Joysticks.getButton(Button.kCross)
                .whenHeld(
                        new StartEndCommand(
                                () -> this.m_Trigger.setLeftAngle(Constants.Trigger.Left.outAngle),
                                () -> this.m_Trigger.setLeftAngle(Constants.Trigger.Left.inAngle)));
        m_Joysticks.getButton(Button.kCircle)
                .whenHeld(
                        new StartEndCommand(
                                () -> this.m_Trigger.setRightAngle(Constants.Trigger.Right.outAngle),
                                () -> this.m_Trigger.setRightAngle(Constants.Trigger.Right.inAngle)));

        m_Joysticks.getButton(Button.kShare)
                .whenPressed(() -> m_Drivetrain.resetYaw());

        m_Joysticks.getButton(Button.kOptions)
                .whenPressed(() -> _fieldRelative = !_fieldRelative); // toggle field relative mode

        m_Joysticks.getButton(Button.kTriangle).whenPressed(
            new ConditionalCommand(
                new InstantCommand(()-> m_arm.getActiveGoalCommand(ArmConstants.shootAngle).schedule()),
                new InstantCommand(()-> m_arm.getActiveGoalCommand(ArmConstants.intakeAngle).schedule()),
                m_arm::isLastGoalIntake));
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

    public void disableInit() {
        m_arm.disabledInit();
    }
=======
  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final Climber m_Climber = new Climber();
  private final Joysticks m_Joysticks = new Joysticks();
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
    this.m_Joysticks.getButton(Button.kSquare).toggleWhenPressed(new StartEndCommand(
      () -> this.m_Climber.setPosition(ClimberState.UP), 
      () -> this.m_Climber.setPosition(ClimberState.DOWN), 
      m_Climber));
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
>>>>>>> 0d31a10 (added joysticks control)
}
