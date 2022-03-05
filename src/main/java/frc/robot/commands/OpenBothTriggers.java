// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.subsystems.trigger.Trigger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OpenBothTriggers extends SequentialCommandGroup {
    /** Creates a new OpenBothTriggers. */
    public OpenBothTriggers(Trigger trigger) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new StartEndCommand(
                        () -> trigger.setBothAngle(Constants.Trigger.Left.outAngle, Constants.Trigger.Right.outAngle),
                        () -> trigger.setBothAngle(Constants.Trigger.Left.inAngle, Constants.Trigger.Right.inAngle),
                        trigger));
    }
}
