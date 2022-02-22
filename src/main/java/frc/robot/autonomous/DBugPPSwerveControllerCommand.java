package frc.robot.autonomous;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * DBugPPSwerveControllerCommand
 */
public class DBugPPSwerveControllerCommand extends PPSwerveControllerCommand {
    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;

    public DBugPPSwerveControllerCommand(PathPlannerTrajectory trajectory, Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics, PIDController xController, PIDController yController,
            ProfiledPIDController thetaController, Consumer<SwerveModuleState[]> outputModuleStates,
            Subsystem... requirements) {
        super(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates,
                requirements);
        m_trajectory = trajectory;
        m_pose = pose;
    }

    @Override
    public void execute() {
        super.execute();
        Pose2d pose = m_pose.get();
        PathPlannerState desiredState = (PathPlannerState) m_trajectory.sample(m_timer.get());

        System.out.printf("%f %f %f %f %f %f\n",
                pose.getX(),
                desiredState.poseMeters.getX(),
                pose.getY(),
                desiredState.poseMeters.getY(),
                pose.getRotation().getDegrees(),
                desiredState.holonomicRotation.getDegrees());
    }

    @Override
    public void initialize() {
        super.initialize();
        m_timer.reset();
        m_timer.start();
    }

}