// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motors.units.PositionUnit;
import frc.robot.subsystems.Arm;

public class ArmToPosition extends CommandBase {
  private Arm _arm;
  private double _goal;
  /** Creates a new ArmToPosition. */
  public ArmToPosition(Arm arm, double goal) {
    addRequirements(arm);
    _arm = arm;
    _goal = goal;
  }

  public ArmToPosition(Arm arm, double goalInDegOrRots, PositionUnit units) {
    this(arm,unitsToRotations(goalInDegOrRots,units));
  }

  public ArmToPosition(Arm arm, Arm.ArmState goalState) {
    this(arm, goalState.getRotations());
  }

  private static double unitsToRotations(double valueDegorRot, PositionUnit units) {
    switch (units) {
      case Rotations: 
        return valueDegorRot;
      case Degrees:
        return valueDegorRot/360;
      default: 
      return 0;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _arm.setGoal(_goal);
    _arm.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _arm.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _arm.reachedEnd(_goal);
  }
}
