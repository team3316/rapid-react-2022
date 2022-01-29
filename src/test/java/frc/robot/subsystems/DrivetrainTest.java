package frc.robot.subsystems;

import static org.junit.Assert.assertArrayEquals;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.utils.Utils;

public class DrivetrainTest {
    static Drivetrain drivetrain;
    static SwerveModule[] modules;
    static PigeonIMU pigeon;

    @BeforeClass
    public static void init() {
        drivetrain = new Drivetrain();
        modules = (SwerveModule[]) Utils.GetPrivate(drivetrain, "_modules");
        pigeon = (PigeonIMU) Utils.ReflectAndSpy(drivetrain, "_pigeon");
    }

    @AfterClass
    public static void close() {
        drivetrain.close();
    }

    @Test
    public void driveNotFieldRelative() {
        drivetrain.drive(1, 0, 0, false);
        assertArrayEquals(
                new double[][] {
                        { 1, 0 },
                        { 1, 0 },
                        { 1, 0 },
                        { 1, 0 }
                },
                getSetPoints());

        drivetrain.drive(0, 2, 0, false);
        assertArrayEquals(
                new double[][] {
                        { 2, 90 },
                        { 2, 90 },
                        { 2, 90 },
                        { 2, 90 }
                },
                getSetPoints());

        drivetrain.drive(1, 1, 0, false);
        assertArrayEquals(

                new double[][] {
                        { Math.sqrt(2), 45 },
                        { Math.sqrt(2), 45 },
                        { Math.sqrt(2), 45 },
                        { Math.sqrt(2), 45 }
                },
                getSetPoints());

        drivetrain.drive(0, 0, 1, false);
        assertArrayEquals(
                flatten(new double[][] {
                        {
                                -0.42119,
                                -52.7233
                        },
                        {
                                -0.42119,
                                52.7233
                        },
                        {
                                0.42119,
                                52.7233
                        },
                        {
                                0.42119,
                                -52.7233
                        }
                }),
                flatten(getSetPoints()),
                0.0001);

    }

    @Test
    public void driveFieldRelative() {
        when(pigeon.getState()).thenReturn(PigeonState.Ready);

        when(pigeon.getFusedHeading()).thenReturn(0.0);
        drivetrain.drive(1, 0, 0, true);
        assertArrayEquals(
                new double[][] {
                        { 1, 0 },
                        { 1, 0 },
                        { 1, 0 },
                        { 1, 0 }
                },
                getSetPoints());

        when(pigeon.getFusedHeading()).thenReturn(90.0);
        drivetrain.drive(1, 0, 0, true);
        assertArrayEquals(
                new double[][] {
                        { 1, -90 },
                        { 1, -90 },
                        { 1, -90 },
                        { 1, -90 }
                },
                getSetPoints());

        when(pigeon.getFusedHeading()).thenReturn(-45.0);
        drivetrain.drive(1, 0, 0, true);
        assertArrayEquals(
                new double[][] {
                        { 1, 45 },
                        { 1, 45 },
                        { 1, 45 },
                        { 1, 45 }
                },
                getSetPoints());

        when(pigeon.getFusedHeading()).thenReturn(180.0);
        drivetrain.drive(1, 0, 0, true);
        assertArrayEquals(
                new double[][] {
                        { -1, 0 },
                        { -1, 0 },
                        { -1, 0 },
                        { -1, 0 }
                },
                getSetPoints());

        when(pigeon.getFusedHeading()).thenReturn(-135.0);
        drivetrain.drive(1, 0, 0, true);
        assertArrayEquals(
                new double[][] {
                        { -1, -45 },
                        { -1, -45 },
                        { -1, -45 },
                        { -1, -45 }
                },
                getSetPoints());

        when(pigeon.getFusedHeading()).thenReturn(-45.0);
        drivetrain.drive(0, 1, 0, true);
        assertArrayEquals(
                new double[][] {
                        { -1, -45 },
                        { -1, -45 },
                        { -1, -45 },
                        { -1, -45 }
                },
                getSetPoints());

    }

    private double[][] getSetPoints() {
        double[][] states = new double[modules.length][2];

        for (int i = 0; i < modules.length; i++) {
            states[i][0] = modules[i].getDriveSetpoint();
            states[i][1] = modules[i].getSteeringSetpoint();
        }
        return states;
    }

    private double[] flatten(double[][] arr) {
        double[] out = new double[arr.length * arr[0].length];
        for (int i = 0; i < arr.length; i++) {
            for (int j = 0; j < arr[0].length; j++) {
                out[i * arr[0].length + j] = arr[i][j];
            }
        }
        return out;
    }

}
