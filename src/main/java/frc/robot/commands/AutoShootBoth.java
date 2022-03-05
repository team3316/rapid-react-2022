// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.trigger.Trigger;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootBoth extends SequentialCommandGroup {
    /** Creates a new AutoShootBoth. */
    public AutoShootBoth(Manipulator manipulator, Trigger trigger) {
        // Add your commands in the addCommands() call, e.g6
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(new InstantCommand(() -> manipulator.setState(ManipulatorState.SHOOT), manipulator),

                new OpenBothTriggers(trigger).withTimeout(0.8),

                new InstantCommand(() -> manipulator.setState(ManipulatorState.OFF), manipulator));
    }
}
