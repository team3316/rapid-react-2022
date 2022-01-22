// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.autonomous.SwerveTrajectoryFollower;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FollowTrajectory extends CommandBase {

  private Drivetrain _drivetrain;
  private SwerveTrajectoryFollower _follower;
  private Trajectory _trajectory;

  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
    _drivetrain = drivetrain;
    _trajectory = trajectory;
    _follower = getFollower(_trajectory);
    addRequirements(_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _drivetrain.resetOdometry(_trajectory.getInitialPose());
    _follower.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _follower.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _follower.isFinished();
  }

  private SwerveTrajectoryFollower getFollower(Trajectory trajectory) {

    var thetaController = new ProfiledPIDController(Constants.Autonomous.kPThetaController, 0, 0,
        Constants.Autonomous.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveTrajectoryFollower swerveTrajectoryFollower = new SwerveTrajectoryFollower(trajectory,
        _drivetrain::getPose, Constants.Drivetrain.kinematics,
        new PIDController(Constants.Autonomous.kPXController, 0, 0),
        new PIDController(Constants.Autonomous.kPYController, 0, 0), thetaController, _drivetrain::setDesiredStates);

    return swerveTrajectoryFollower;
  }

}
