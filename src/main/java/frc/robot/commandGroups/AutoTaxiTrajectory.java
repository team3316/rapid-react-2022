// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowTrajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoTaxiTrajectory extends SequentialCommandGroup {

    public AutoTaxiTrajectory(Drivetrain drivetrain) {

        FollowTrajectory taxi_path = new FollowTrajectory(drivetrain, "taxi");

        addCommands(
                taxi_path.getResetOddometryCommand(),
                taxi_path.getFollowTrajectoryCommand());
    }
}
