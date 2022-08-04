package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowTrajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoStraightPath extends SequentialCommandGroup {
    public AutoStraightPath(Drivetrain drivetrain) {
        FollowTrajectory taxi_path = new FollowTrajectory(drivetrain, "JuneIsOver");

        addCommands(
                taxi_path.getResetOddometryCommand(),
                taxi_path.getFollowTrajectoryCommand());
    }
}
