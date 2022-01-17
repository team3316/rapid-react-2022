// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.commands.FollowTrajectory;
import frc.robot.humanIO.Joysticks;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_Drivetrain = new Drivetrain();

  private final Joysticks m_Joysticks = new Joysticks();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    m_Drivetrain.setDefaultCommand(
      new RunCommand(
        () ->
        m_Drivetrain.drive(
                m_Joysticks.getDriveX() * SwerveModuleConstants.freeSpeedMetersPerSecond,
                m_Joysticks.getDriveY() * SwerveModuleConstants.freeSpeedMetersPerSecond,
                m_Joysticks.getSteerX() * 11.5,
                true),
        m_Drivetrain));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_Joysticks.getButton(Button.kBack).whenPressed(() -> m_Drivetrain.resetYaw());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new FollowTrajectory(m_Drivetrain, getTrajectory());
  }

  
  private static Trajectory getTrajectory() {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Autonomous.kMaxSpeedMetersPerSecond,
        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Drivetrain.kinematics);

    // An example trajectory to follow. All units in meters.
    return TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0.5), new Translation2d(2, -0.5)),
        // List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3
        , 0, Rotation2d.fromDegrees(0)), config);

  }
}
