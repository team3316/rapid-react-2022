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
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.trigger.Trigger;

public class ShootCollectShoot extends SequentialCommandGroup {

    public ShootCollectShoot(Manipulator manipulator, Trigger trigger, Arm arm, FollowTrajectory followTrajectory,
            FollowTrajectory followTrajectoryReverse) {

        addCommands(
                new AutonomousShoot(arm, manipulator, trigger),
                new InstantCommand(() -> arm.getActiveGoalCommand(Constants.ArmConstants.intakeAngle).schedule()),
                new WaitUntilCommand(arm::atGoal),
                followTrajectory.getFollowTrajectoryCommand().deadlineWith(new Collect(manipulator)),
                followTrajectoryReverse.getFollowTrajectoryCommand(),
                new AutonomousShoot(arm, manipulator, trigger));
    }
}
