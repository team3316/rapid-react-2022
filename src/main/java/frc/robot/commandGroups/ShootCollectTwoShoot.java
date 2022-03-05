// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.autonomous.FollowTrajectory;
import frc.robot.commands.Collect;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.trigger.Trigger;

public class ShootCollectTwoShoot extends SequentialCommandGroup {

    public ShootCollectTwoShoot(Drivetrain drivetrain, Arm arm, Manipulator manipulator, Trigger trigger) {

        FollowTrajectory path1 = new FollowTrajectory(drivetrain, "collect_two_path1");
        FollowTrajectory path2 = new FollowTrajectory(drivetrain, "collect_two_path2");
        FollowTrajectory path3 = new FollowTrajectory(drivetrain, "collect_two_path3");

        addCommands(new AutonomousShoot(arm, manipulator, trigger),
                new InstantCommand(() -> arm.getActiveGoalCommand(Constants.ArmConstants.intakeAngle).schedule()),
                new WaitUntilCommand(arm::atGoal),
                sequence(path1.getFollowTrajectoryCommand().beforeStarting(path1.getResetOddometryCommand()),
                        path2.getFollowTrajectoryCommand()).deadlineWith(new Collect(manipulator)),
                path3.getFollowTrajectoryCommand(),
                new AutonomousShoot(arm, manipulator, trigger));
    }
}
