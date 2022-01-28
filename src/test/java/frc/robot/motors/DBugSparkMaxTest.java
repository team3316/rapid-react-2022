package frc.robot.motors;

import static org.junit.Assert.assertEquals;

import org.junit.BeforeClass;
import org.junit.Test;

import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

public class DBugSparkMaxTest {
    static DBugSparkMax _spark1;
    static DBugSparkMax _spark2;

    @BeforeClass
    public static void init() {
        _spark1 = new DBugSparkMax(1, new UnitConversions(2, 1, 1));
        _spark2 = new DBugSparkMax(2, new UnitConversions(1, 1, 1));
    }

    @Test
    public void set() {
        _spark1.set(ControlMode.Velocity, 100, VelocityUnit.RPM);
        assertEquals(100 / 2, _spark1.getEncoder().getVelocity(), 0.0000001);
        assertEquals(100.0 / 60 * Math.PI, _spark1.getVelocity(VelocityUnit.MetersPerSecond), 0.0000001);
        _spark1.set(0);
    }

    @Test
    public void followAndInvert() {
        // TODO: Check IRL, I suspect simulated sparkmax do not support this
        // _spark2.setInverted(true);
        // _spark2.follow((IDBugMotorController)_spark1);
        // _spark1.set(ControlMode.PercentOutput, 0.4);
        // assertEquals(-0.4, _spark2.get(), 0.0000001);
    }
}
