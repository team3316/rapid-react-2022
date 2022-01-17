package frc.robot.autonomous;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

public class SwerveTrajectoryFollower {
    private final Timer _timer = new Timer();
    private HolonomicDriveController _controller;
    private SwerveDriveKinematics _kinematics;
    private Trajectory _trajectory;
    private Consumer<SwerveModuleState[]> _outputModuleStates;
    private Supplier<Pose2d> _pose;

    public SwerveTrajectoryFollower(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
            PIDController xController, PIDController yController, ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates) {
        _kinematics = kinematics;
        _trajectory = trajectory;
        _pose = pose;
        _outputModuleStates = outputModuleStates;

        _controller = new HolonomicDriveController(xController, yController, thetaController);
    }

    public void initialize() {
        _timer.reset();
        _timer.start();
    }

    public void execute() {
        double curTime = _timer.get();
        var desiredState = _trajectory.sample(curTime);
        var angleRef = _trajectory.getStates().get(_trajectory.getStates().size() - 1).poseMeters.getRotation();

        var targetChassisSpeeds = _controller.calculate(_pose.get(), desiredState, angleRef);
        var targetModuleStates = _kinematics.toSwerveModuleStates(targetChassisSpeeds);

        _outputModuleStates.accept(targetModuleStates);
    }

    public boolean isFinished() {
        return _timer.hasElapsed(_trajectory.getTotalTimeSeconds());
    }
}
