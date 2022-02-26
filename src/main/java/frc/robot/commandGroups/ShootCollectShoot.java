// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autonomous.FollowTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystems.trigger.Trigger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCollectShoot extends SequentialCommandGroup {
  /** Creates a new ShootCollect. */
  public ShootCollectShoot(Manipulator manipulator, Trigger trigger, Arm arm, FollowTrajectory followTrajectory, FollowTrajectory followTrajectoryReverse) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(arm.getActiveGoalCommand(Constants.ArmConstants.shootAngle),
                new AutoShoot(manipulator, trigger), 
                arm.getActiveGoalCommand(Constants.ArmConstants.startingAngle),
                raceWith(new InstantCommand(() -> manipulator.setState(ManipulatorState.COLLECT)),
                        followTrajectory.getFollowTrajectoryCommand()),
                followTrajectoryReverse.getFollowTrajectoryCommand(),
                arm.getActiveGoalCommand(Constants.ArmConstants.shootAngle),
                new AutoShoot(manipulator, trigger));
  }
}
