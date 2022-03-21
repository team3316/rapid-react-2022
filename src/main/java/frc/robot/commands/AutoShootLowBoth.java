// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.trigger.Trigger;
import frc.robot.Constants.LED.RobotColorState;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;

public class AutoShootLowBoth extends SequentialCommandGroup {
    public AutoShootLowBoth(Manipulator manipulator, Trigger trigger, LED led) {
        addCommands(new InstantCommand(() -> manipulator.setState(ManipulatorState.SHOOTLOW), manipulator),

                new OpenBothTriggers(trigger).withTimeout(0.8),

                new InstantCommand(() -> manipulator.setState(ManipulatorState.OFF), manipulator),
                
                new InstantCommand(() -> led.setRobotColor(RobotColorState.COLLECT)));
    }
}