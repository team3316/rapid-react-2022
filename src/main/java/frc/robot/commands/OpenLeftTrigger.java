// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.subsystems.trigger.Trigger;

public class OpenLeftTrigger extends SequentialCommandGroup {
    /** Creates a new SetRightTrigger. */
    public OpenLeftTrigger(Trigger trigger) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(

                new StartEndCommand(
                        () -> trigger.setLeftAngle(Constants.Trigger.Left.outAngle),
                        () -> trigger.setLeftAngle(Constants.Trigger.Left.inAngle), trigger));
    }
}
