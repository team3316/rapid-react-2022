// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmTest extends CommandBase {
  private Arm _arm;
  private double _prcnt;
  /** Creates a new ArmToPosition. */
  public ArmTest(Arm arm, double prcnt) {
    addRequirements(arm);
    _arm = arm;
    _prcnt = prcnt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      _arm.testState(_prcnt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _arm.testState(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //what should this value be?
  }
}
