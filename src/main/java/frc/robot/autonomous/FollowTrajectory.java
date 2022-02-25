// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FollowTrajectory extends SubsystemBase {
    private final Drivetrain m_drivetrain;
    private PathPlannerTrajectory m_trajectory;
    private PIDController _xController;
    private PIDController _yController;
    private TrajectoryPIDController _thetaController;
    
    private SendableChooser<String> _chooser;

    public FollowTrajectory(Drivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
        this.m_trajectory = PathPlanner.loadPath(Constants.Autonomous.defaultPath,
                Constants.Autonomous.kMaxSpeedMetersPerSecond,
                Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);
        this._xController = new PIDController(Constants.Autonomous.kPXController, 0, 0);
        this._yController = new PIDController(Constants.Autonomous.kPYController, 0, 0);
        this._thetaController = new TrajectoryPIDController(Constants.Autonomous.gains);
        initSDB();
    }

    private void initSDB() {
        SmartDashboard.setDefaultNumber("Auto Max Velocity", Constants.Autonomous.kMaxSpeedMetersPerSecond);
        SmartDashboard.setDefaultNumber("Auto Max Accleration",
                Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);

        SmartDashboard.putData("Update Auto Trajectory", new InstantCommand(() -> updateTrajectory()));

        this._chooser = new SendableChooser<String>();
        _chooser.setDefaultOption("straight path", "straight_path");
        _chooser.addOption("s path", "s");
        _chooser.addOption("180 rotation path", "rotate");
        // Add a new autonomous path:
        // _chooser.addOption("straight path", "straight_path");

        SmartDashboard.putData("paths chooser", this._chooser);
    }

    public void periodic() {
    }

    private void updateTrajectory() {
        this.m_trajectory = PathPlanner.loadPath(this._chooser.getSelected(),
                SmartDashboard.getNumber("Auto max velocity", Constants.Autonomous.kMaxSpeedMetersPerSecond),
                SmartDashboard.getNumber("Auto Max Accleration",
                        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared));
    }

    public Command getFollowTrajectoryCommand() {
        PathPlannerState initState = m_trajectory.getInitialState();

        return new DBugPPSwerveControllerCommand(m_trajectory, _xController, _yController,
                _thetaController, m_drivetrain)
                        .beforeStarting(() -> m_drivetrain.resetOdometry(
                                new Pose2d(initState.poseMeters.getTranslation(), initState.holonomicRotation)));
    }
}
