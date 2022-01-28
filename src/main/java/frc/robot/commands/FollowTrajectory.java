// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FollowTrajectory extends CommandBase {
    private final Timer _timer = new Timer();
    private HolonomicDriveController _controller;
    private SwerveDriveKinematics _kinematics;
    private Trajectory _trajectory;
    private Consumer<SwerveModuleState[]> _outputModuleStates;
    private Supplier<Pose2d> _pose;
    private Rotation2d _angleRef;
    private final Drivetrain _drivetrain;

    /** Creates a new FollowTrajectoryTemp. */
    public FollowTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
        this._drivetrain = drivetrain;
        this._kinematics = Constants.Drivetrain.kinematics;
        this._trajectory = trajectory;
        _pose = this._drivetrain::getPose;
        _outputModuleStates = this._drivetrain::setDesiredStates;
        _angleRef = _trajectory.getStates().get(_trajectory.getStates().size() - 1).poseMeters.getRotation();

        var thetaController = new ProfiledPIDController(Constants.Autonomous.kPThetaController, 0, 0,
                Constants.Autonomous.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        _controller = new HolonomicDriveController(
                new PIDController(Constants.Autonomous.kPXController, 0, 0),
                new PIDController(Constants.Autonomous.kPYController, 0, 0),
                thetaController);

        addRequirements(this._drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _drivetrain.resetOdometry(_trajectory.getInitialPose());
        _timer.reset();
        _timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double curTime = _timer.get();
        var desiredState = _trajectory.sample(curTime);
        var pose = _pose.get();

        var targetChassisSpeeds = _controller.calculate(pose, desiredState, _angleRef);
        targetChassisSpeeds.omegaRadiansPerSecond *= -1;
        System.out.printf("%f %f %f %f %f %f %f %f %f\n",
                pose.getX(),
                desiredState.poseMeters.getX(),
                pose.getY(),
                desiredState.poseMeters.getY(),
                pose.getRotation().getDegrees(),
                _angleRef.getDegrees(),
                Math.hypot(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond),
                desiredState.velocityMetersPerSecond);

        var targetModuleStates = _kinematics.toSwerveModuleStates(targetChassisSpeeds);

        _outputModuleStates.accept(targetModuleStates);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _drivetrain.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return _timer.hasElapsed(_trajectory.getTotalTimeSeconds());
    }
}
