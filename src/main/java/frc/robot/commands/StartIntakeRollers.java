// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Intake;

public class StartIntakeRollers extends CommandBase {
private final Intake _intake;

  /** Creates a new StartIntakeRollers. */
  public StartIntakeRollers(Intake intake) {
    _intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _intake.setRPM(Constants.Intake.RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intake.setZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _intake.getCargoState().hasBoth();
  }
}
