// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.utils.Within;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbHigh extends SequentialCommandGroup {
    /** Creates a new ClimbHigh. */
    public ClimbHigh(Climber climber) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        addRequirements(climber);
        addCommands(
                new InstantCommand(() -> climber.setMid(-1)),
                new WaitUntilCommand(
                        () -> Within.range(climber.getLeftPosition(), 0.1, 0.02)),
                new InstantCommand(() -> climber.setMid(-0.4)),
                new WaitUntilCommand(
                        () -> Within.range(climber.getLeftPosition(), Constants.Climber.startingPosition, 0.04)),
                new InstantCommand(() -> climber.setHigh(1)),
                new WaitUntilCommand(() -> Within.range(climber.getHighPosition(),
                        Constants.Climber.highExtentionHeight, 0.02)),
                new InstantCommand(() -> climber.setMid(0.4)),
                new WaitUntilCommand(
                        () -> Within.range(climber.getLeftPosition(), 0.30, 0.02)),
                new InstantCommand(() -> climber.setMid(0.2)),
                new InstantCommand(() -> climber.setHigh(0.1)),
                new WaitUntilCommand(
                        () -> Within.range(climber.getLeftPosition(), 0.50, 0.02)),
                new InstantCommand(() -> climber.setMid(0.5)),
                new InstantCommand(() -> climber.setHigh(-1)),
                new WaitCommand(3)
        // new InstantCommand(() -> climber.set(0)),
        // new InstantCommand(() -> climber.setHighClimb(-0.7))
        );
    }
}
