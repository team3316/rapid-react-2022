// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm.subsystems.Arm;
import frc.robot.Arm.subsystems.Arm.ArmState;
import frc.robot.Constants.ArmConstants;

public class ArmToPositionIntake extends CommandBase {
  private Arm _arm;
  private double _goal;
  /** Creates a new ArmToPosition. */
  /**
   * @param arm the subsystem
   * @param goal in Neo Rotations
   */
  public ArmToPositionIntake(Arm arm) {
    addRequirements(arm);
    _arm = arm;
    _goal = ArmState.INTAKE.getRotations();
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _arm.setGoal(_goal*(2*Math.PI)*ArmConstants.gearRatioNeoToArm); //goal: neo rotations to arm rads
    SmartDashboard.putNumber("test", _goal*(2*Math.PI)*ArmConstants.gearRatioNeoToArm);
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
    // TODO: Make this default command
    return false;
  }
}
