package frc.robot.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DBugPPSwerveControllerCommand extends CommandBase {
    private final boolean DEBUG = true;

    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final HolonomicDriveController m_controller;
    private final Drivetrain m_drivetrain;
    private final TrajectoryPIDController m_thetaController;

    public DBugPPSwerveControllerCommand(
            PathPlannerTrajectory trajectory,
            PIDController xController,
            PIDController yController,
            TrajectoryPIDController thetaController,
            Drivetrain drivetrain) {
        m_trajectory = trajectory;
        m_drivetrain = drivetrain;
        m_thetaController = thetaController;

        m_controller = new HolonomicDriveController(
                xController,
                yController,
                thetaController);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        if (DEBUG)
            System.out.printf("time,xError,xP,xF,yError,yP,yF,aError,aP,aI,aD,aF\n");
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);
        Pose2d currentPose = m_drivetrain.getPose();

        var targetChassisSpeeds = m_controller.calculate(currentPose, desiredState, desiredState.holonomicRotation);
        if (DEBUG) {
            // Calculate back the constituate parts of the chassis speeds.
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds.vxMetersPerSecond,
                    targetChassisSpeeds.vyMetersPerSecond, targetChassisSpeeds.omegaRadiansPerSecond,
                    currentPose.getRotation().times(-1));
            Pose2d desiredPose = desiredState.poseMeters;
            double xFF = desiredState.velocityMetersPerSecond * desiredPose.getRotation().getCos();
            double yFF = desiredState.velocityMetersPerSecond * desiredPose.getRotation().getSin();
            System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                    curTime,
                    desiredPose.getX() - currentPose.getX(),
                    fieldSpeeds.vxMetersPerSecond - xFF,
                    xFF,
                    desiredPose.getY() - currentPose.getY(),
                    fieldSpeeds.vyMetersPerSecond - yFF,
                    yFF,
                    desiredState.holonomicRotation.getRadians() - currentPose.getRotation().getRadians(),
                    m_thetaController.kPContribution,
                    m_thetaController.kIContribution,
                    m_thetaController.kDContribution,
                    m_thetaController.kFContribution);
        }

        var targetModuleStates = Constants.Drivetrain.kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_drivetrain.setDesiredStates(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
