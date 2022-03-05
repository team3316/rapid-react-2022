// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.subsystems.trigger.Trigger;

public class OpenRightTrigger extends SequentialCommandGroup {
    /** Creates a new SetRightTrigger. */
    public OpenRightTrigger(Trigger trigger) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(

                new StartEndCommand(
                        () -> trigger.setRightAngle(Constants.Trigger.Right.outAngle),
                        () -> trigger.setRightAngle(Constants.Trigger.Right.inAngle)));
    }
}
