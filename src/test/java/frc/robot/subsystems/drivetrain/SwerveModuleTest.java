package frc.robot.subsystems.drivetrain;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleTest {

    @Test
    public void compare_optimize() {
        // both directions and stopped
        for (double speedMetersPerSecond = -1; speedMetersPerSecond <= 1; speedMetersPerSecond++) {
            // any desired angle
            for (double desiredAngle = -180; desiredAngle <= 180; desiredAngle++) {
                // -2 to +2 current robot turns
                for (double currentAngle = -720; currentAngle <= 720; currentAngle++) {
                    SwerveModuleState desiredState = new SwerveModuleState(speedMetersPerSecond,
                            Rotation2d.fromDegrees(desiredAngle));
                    SwerveModuleState optimized = SwerveModule.optimize(desiredState, currentAngle);

                    double optimizedTurnAngle = optimized.angle.getDegrees() - currentAngle;
                    assertTrue("working turn is too large", Math.abs(optimizedTurnAngle) <= 90.01);
                }
            }
        }
    }

}
