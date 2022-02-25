package frc.robot.motors;

import static org.junit.Assert.assertEquals;

import org.junit.BeforeClass;
import org.junit.Test;

import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

public class UnitConversionTest {
    static UnitConversions conversions;

    @BeforeClass
    public static void init() {
        conversions = new UnitConversions(0.7, 4, 1024);
    }

    @Test
    public void getRotationsModifier() {
        assertEquals(0.7, conversions.getRotationsModifier(PositionUnit.Rotations), 0.0000001);
        assertEquals(0.7 * 360, conversions.getRotationsModifier(PositionUnit.Degrees), 0.0000001);
        assertEquals(0.7 * 4 * Math.PI, conversions.getRotationsModifier(PositionUnit.Meters), 0.0000001);
    }

    @Test
    public void getRPMModifier() {
        assertEquals(0.7, conversions.getRPMModifier(VelocityUnit.RPM), 0.0000001);
        assertEquals(0.7 / 60 * 4 * Math.PI, conversions.getRPMModifier(VelocityUnit.MetersPerSecond), 0.0000001);
    }
}
