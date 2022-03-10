// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    private Supplier<Pose2d> _pose;
    private SwerveDriveKinematics _kinematics;
    private PIDController _xController;
    private PIDController _yController;
    private ProfiledPIDController _thetaController;
    private Consumer<SwerveModuleState[]> _outputModuleStates;

    private SendableChooser<String> _chooser;

    public FollowTrajectory(Drivetrain drivetrain, String path) {
        this.m_drivetrain = drivetrain;
        this.m_trajectory = PathPlanner.loadPath(path,
                Constants.Autonomous.kMaxSpeedMetersPerSecond,
                Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);
        this._pose = m_drivetrain::getPose;
        this._kinematics = Constants.Drivetrain.kinematics;
        this._xController = new PIDController(Constants.Autonomous.kPXController, 0, 0);
        this._yController = new PIDController(Constants.Autonomous.kPYController, 0, 0);
        this._thetaController = new ProfiledPIDController(Constants.Autonomous.kPThetaController, 0, 0,
                Constants.Autonomous.kThetaControllerConstraints);
        _thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this._outputModuleStates = this.m_drivetrain::setDesiredStates;
        // initSDB();
    }

    private void initSDB() {
        SmartDashboard.setDefaultNumber("P Gain Auto xController", Constants.Autonomous.kPXController);
        SmartDashboard.setDefaultNumber("P Gain Auto yController", Constants.Autonomous.kPYController);
        SmartDashboard.setDefaultNumber("P Gain Auto thetaCotroller", Constants.Autonomous.kPThetaController);
        SmartDashboard.setDefaultNumber("Auto Max Velocity", Constants.Autonomous.kMaxSpeedMetersPerSecond);
        SmartDashboard.setDefaultNumber("Auto Max Accleration",
                Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared);

        SmartDashboard.putData("Update Auto PID", new InstantCommand(() -> updatePID()));
        SmartDashboard.putData("Update Auto Trajectory", new InstantCommand(() -> updateTrajectory()));

        this._chooser = new SendableChooser<String>();
        _chooser.setDefaultOption("straight path", "straight_path");
        _chooser.addOption("s path", "s");
        _chooser.addOption("180 rotation path", "rotate");
        _chooser.addOption("collect 1 CARGO", "collect");
        // Add a new autonomous path:
        // _chooser.addOption("straight path", "straight_path");

        SmartDashboard.putData("paths chooser", this._chooser);
    }

    public void periodic() {
        SmartDashboard.putNumber("x pose", m_drivetrain.getPose().getX());
    }

    private void updatePID() {
        _xController.setP(SmartDashboard.getNumber("P Gain Auto xController", Constants.Autonomous.kPXController));
        _yController.setP(SmartDashboard.getNumber("P Gain Auto yController", Constants.Autonomous.kPYController));
        _thetaController
                .setP(SmartDashboard.getNumber("P Gain Auto thetaCotroller", Constants.Autonomous.kPThetaController));
    }

    private void updateTrajectory() {
        this.m_trajectory = PathPlanner.loadPath(this._chooser.getSelected(),
                SmartDashboard.getNumber("Auto max velocity", Constants.Autonomous.kMaxSpeedMetersPerSecond),
                SmartDashboard.getNumber("Auto Max Accleration",
                        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared));
    }

    public Command getFollowTrajectoryCommand() {

        return new DBugPPSwerveControllerCommand(m_trajectory,
                _pose, _kinematics,
                _xController, _yController,
                _thetaController,
                _outputModuleStates,
                () -> m_trajectory.getEndState().holonomicRotation,
                m_drivetrain);
    }

    public Command getResetOddometryCommand() {
        PathPlannerState initState = m_trajectory.getInitialState();

        return new InstantCommand(() -> {
            m_drivetrain.resetOdometry(
                    new Pose2d(initState.poseMeters.getTranslation(), initState.holonomicRotation));
            m_drivetrain.setYaw(initState.holonomicRotation.getDegrees());
        });
    }
}
