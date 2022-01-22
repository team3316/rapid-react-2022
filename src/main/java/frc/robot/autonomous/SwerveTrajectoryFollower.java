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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        var pose = _pose.get();

        var targetChassisSpeeds = _controller.calculate(pose, desiredState, angleRef);
        targetChassisSpeeds.omegaRadiansPerSecond *= -1;
        System.out.printf("%f %f %f %f %f %f %f %f %f\n", pose.getX(), 
                                                          desiredState.poseMeters.getX(),
                                                          pose.getY(), 
                                                          desiredState.poseMeters.getY(),
                                                          pose.getRotation().getDegrees(), 
                                                          angleRef.getDegrees(),
                                                          Math.hypot(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond),
                                                          desiredState.velocityMetersPerSecond);
                                                        
        var targetModuleStates = _kinematics.toSwerveModuleStates(targetChassisSpeeds);
        
        _outputModuleStates.accept(targetModuleStates);
    }

    public boolean isFinished() {
        return _timer.hasElapsed(_trajectory.getTotalTimeSeconds());
    }
}
