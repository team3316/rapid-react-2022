// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoShootFromRobot;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystems.trigger.Trigger;

public class AutonomousShootFromRobot extends SequentialCommandGroup {

    public AutonomousShootFromRobot(Arm arm, Manipulator manipulator, Trigger trigger, boolean shootLeft,
            boolean shootRight, LED led) {

        addCommands(
                new InstantCommand(() -> manipulator.setState(ManipulatorState.KEEPIN)),
                new InstantCommand(() -> arm.getActiveGoalCommand(Constants.ArmConstants.shootAngle, led).schedule()),
                new WaitUntilCommand(arm::atGoal),
                new AutoShootFromRobot(manipulator, trigger, shootLeft, shootRight, led));
    }

    public AutonomousShootFromRobot(Arm arm, Manipulator manipulator, Trigger trigger, LED led) {
        this(arm, manipulator, trigger, true, true, led);
    }
}
