package frc.robot.subsystems;

import org.junit.AfterClass;
import org.junit.BeforeClass;

import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class SwerveModuleTest {
    static SwerveModule module;

    @BeforeClass
    public static void init() {
        module = new SwerveModule(Drivetrain.TRModule);
    }

    @AfterClass
    public static void close() {
        module.close();
    }

    // @Test
    // public void setState() {
    //     var state = new SwerveModuleState(1,Rotation2d.fromDegrees(90));
    //     module.setDesiredState(state);
    //     assertEquals(state.toString(), module.getState());
    // }
}
