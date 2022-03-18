// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.trigger.Trigger;

public class OpenTriggersWithDelay extends SequentialCommandGroup {
    boolean _isFinished = false;

    public OpenTriggersWithDelay(Trigger trigger, boolean shootLeft, boolean shootRight) {
        SequentialCommandGroup _sequence = new SequentialCommandGroup();
        if (shootRight)
            _sequence.addCommands(
                    new InstantCommand(
                            () -> trigger.setRightAngle(Constants.Trigger.Right.outAngle)),
                    new WaitCommand(0.5));

        if (shootLeft)
            _sequence.addCommands(
                    new InstantCommand(
                            () -> trigger.setLeftAngle(Constants.Trigger.Left.outAngle)),
                    new WaitCommand(0.5));

        if (shootRight || shootLeft)
            _sequence.addCommands(new WaitCommand(0.5), new InstantCommand(() -> {
                this._isFinished = true;
            }));

        addCommands(
                new FunctionalCommand(
                        () -> {
                            _isFinished = false;
                            _sequence.schedule();
                        },
                        () -> {
                        },
                        (interrupted) -> {DataLogManager.log("shooting interrupted: " + interrupted);
                            trigger.setRightAngle(Constants.Trigger.Right.inAngle);
                            trigger.setLeftAngle(Constants.Trigger.Left.inAngle);
                        },
                        () -> {
                            if (this._isFinished) {
                                this._isFinished = false;
                                return true;
                            }
                            return false;
                        }));
    }
}
