// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystems.trigger.Trigger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
    /** Creates a new AutoShoot. */
    public AutoShoot(Manipulator manipulator, Trigger trigger) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new InstantCommand(() -> manipulator.setState(ManipulatorState.SHOOT), manipulator),
                new WaitCommand(1.3),
                new InstantCommand(() -> trigger.setLeftAngle(Constants.Trigger.Left.outAngle)),
                new WaitCommand(0.3),
                parallel(
                        new InstantCommand(() -> trigger.setLeftAngle(Constants.Trigger.Left.inAngle)),
                        new InstantCommand(() -> trigger.setRightAngle(Constants.Trigger.Right.outAngle))),
                new WaitCommand(0.3),
                new InstantCommand(() -> trigger.setRightAngle(Constants.Trigger.Right.inAngle)),
                new InstantCommand(() -> manipulator.setState(ManipulatorState.OFF), manipulator));
    }
}
