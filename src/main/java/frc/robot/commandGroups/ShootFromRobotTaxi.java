// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.autonomous.FollowTrajectory;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.trigger.Trigger;

public class ShootFromRobotTaxi extends SequentialCommandGroup {

    public ShootFromRobotTaxi(Drivetrain drivetrain, Manipulator manipulator, Trigger trigger, Arm arm, LED led) {

        FollowTrajectory taxi_path = new FollowTrajectory(drivetrain, "taxi");

        addCommands(
                new AutonomousShootFromRobot(arm, manipulator, trigger, false, true, led),
                new InstantCommand(() -> arm.getActiveGoalCommand(Constants.ArmConstants.intakeAngle, led).schedule()),
                new WaitUntilCommand(arm::atGoal),
                taxi_path.getResetOddometryCommand(),
                taxi_path.getFollowTrajectoryCommand());
    }
}
