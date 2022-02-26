// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;

public class SetManipulatorState extends CommandBase {
  
    private Manipulator m_manipulator;
    private ManipulatorState wanted_state;

  public SetManipulatorState(Manipulator manipulator, ManipulatorState state) {
      this.wanted_state = state;
      this.m_manipulator = manipulator;
    addRequirements(m_manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_manipulator.setState(wanted_state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      this.m_manipulator.setState(ManipulatorState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
