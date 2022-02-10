// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystems.trigger.Trigger;

/**
 * Currently untested
 */
public class Shoot extends CommandBase {

    private final Manipulator m_Manipulator;

    private final Trigger m_Trigger;

    private int _shots = 0;

    public Shoot(Manipulator manipulator, Trigger trigger) {
        m_Manipulator = manipulator;
        m_Trigger = trigger;
        addRequirements(m_Manipulator, m_Trigger);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Manipulator.setState(ManipulatorState.SHOOT);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_Manipulator.atSetpoint()) {
            switch (_shots) {
                case 0:
                    _shots++;
                    m_Trigger.setLeftAngle(Constants.Trigger.Left.outAngle);
                    break;
                case 1:
                    _shots++;
                    m_Trigger.setLeftAngle(Constants.Trigger.Left.inAngle);
                    m_Trigger.setRightAngle(Constants.Trigger.Right.outAngle);
                default:
                    break;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Trigger.setRightAngle(Constants.Trigger.Right.inAngle);
        m_Manipulator.setState(ManipulatorState.OFF);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return _shots == 2 && m_Manipulator.atSetpoint();
    }
}
