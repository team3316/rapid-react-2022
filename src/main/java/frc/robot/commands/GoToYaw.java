// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class GoToYaw extends CommandBase {
  /** Creates a new GoToYaw. */
  private final double _setpoint;
  private final PIDController _controller = new PIDController(20 * 0.3, 0, 0.0);
  private final Drivetrain _drivetrain;
  private final SlewRateLimiter _limiter = new SlewRateLimiter(20);

  public GoToYaw(Drivetrain drivetrain, double angle) {
    _setpoint = Math.toRadians(angle);
    _drivetrain = drivetrain;
    _controller.setTolerance(0.01 / 0.3);
    // _controller.enableContinuousInput(-180, 180);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _controller.setSetpoint(_drivetrain.getRotation2d().getRadians() + _setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pos = _drivetrain.getRotation2d().getRadians();
    double output = _controller.calculate(pos);
    SmartDashboard.putNumber("pos", pos);
    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("setpoint", _controller.getSetpoint());

    double limitedOutput = _limiter.calculate(output);
    SmartDashboard.putNumber("limited", limitedOutput);
    

    _drivetrain.setDesiredStates(
        Constants.Drivetrain.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, limitedOutput)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _controller.atSetpoint();
  }
}
