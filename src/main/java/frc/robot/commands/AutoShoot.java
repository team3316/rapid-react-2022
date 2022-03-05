// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystems.trigger.Trigger;

public class AutoShoot extends SequentialCommandGroup {

    public AutoShoot(Manipulator manipulator, Trigger trigger) {

        addCommands(
                new InstantCommand(() -> manipulator.setState(ManipulatorState.SHOOT), manipulator),
                new WaitCommand(0.3),

                new OpenRightTrigger(trigger).withTimeout(0.6),

                new OpenLeftTrigger(trigger).withTimeout(0.6),

                new InstantCommand(() -> manipulator.setState(ManipulatorState.OFF), manipulator));
    }
}
