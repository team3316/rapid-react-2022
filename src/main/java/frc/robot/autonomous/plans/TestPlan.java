// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.plans;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestPlan extends SequentialCommandGroup {
  /** Creates a new TestPlan. */
  public TestPlan(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FollowTrajectory(drivetrain, getTrajectory()));
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
        List.of(/*new Translation2d(1, 1)*/),
        // List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2, 0, Rotation2d.fromDegrees(90)), config);

  }
}