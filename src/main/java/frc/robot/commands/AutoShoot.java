// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystems.trigger.Trigger;

public class AutoShoot extends SequentialCommandGroup {

    public AutoShoot(Manipulator manipulator, Trigger trigger) {

        addCommands(
                new InstantCommand(() -> manipulator.setState(ManipulatorState.SHOOT), manipulator),
                new WaitCommand(1.3),

                new ConditionalCommand(
                        sequence(
                                new InstantCommand(() -> trigger.setLeftAngle(Constants.Trigger.Left.outAngle)),
                                new WaitCommand(0.6),
                                new InstantCommand(() -> trigger.setLeftAngle(Constants.Trigger.Left.inAngle))),
                        new InstantCommand(),
                        () -> manipulator.getCargoState().leftCargo),

                new ConditionalCommand(
                        sequence(
                                new InstantCommand(() -> trigger.setRightAngle(Constants.Trigger.Right.outAngle)),
                                new WaitCommand(0.6),
                                new InstantCommand(() -> trigger.setRightAngle(Constants.Trigger.Right.inAngle))),
                        new InstantCommand(),
                        () -> manipulator.getCargoState().rightCargo),

                new InstantCommand(() -> manipulator.setState(ManipulatorState.OFF), manipulator));
    }
}
