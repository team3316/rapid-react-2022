package frc.robot.motors;

import static org.junit.Assert.assertEquals;

import org.junit.BeforeClass;
import org.junit.Test;

import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;

public class DBugServoTest {
    static DBugServo _servo1;
    static DBugServo _servo2;
    static DBugServo _servo3;

    @BeforeClass
    public static void init() {
        _servo1 = new DBugServo(4, new UnitConversions(1, 1, 1));
        _servo2 = new DBugServo(5, new UnitConversions(1, 1, 1));
        _servo3 = new DBugServo(6, new UnitConversions(1, 1, 1));
    }

    @Test
    public void set() {
        unfollow();
        _servo1.set(ControlMode.Position, 90, PositionUnit.Degrees);
        assertEquals(90, _servo1.getPosition(PositionUnit.Degrees), 0.0000001);
    }

    @Test
    public void follow() {
        unfollow();

        _servo2.follow(_servo1);
        _servo3.follow(_servo1);
        _servo1.set(ControlMode.Position, 60, PositionUnit.Degrees);
        assertEquals(60, _servo1.getPosition(PositionUnit.Degrees), 0.0000001);
        assertEquals(60, _servo2.getPosition(PositionUnit.Degrees), 0.0000001);
        assertEquals(60, _servo3.getPosition(PositionUnit.Degrees), 0.0000001);

        _servo2.follow(_servo3);

        _servo1.set(ControlMode.Position, 10, PositionUnit.Degrees);
        assertEquals(10, _servo1.getPosition(PositionUnit.Degrees), 0.0000001);
        assertEquals(10, _servo2.getPosition(PositionUnit.Degrees), 0.0000001);
        assertEquals(10, _servo3.getPosition(PositionUnit.Degrees), 0.0000001);

    }

    @Test(expected = IllegalArgumentException.class)
    public void followCircular() {
        unfollow();

        _servo2.follow(_servo1);
        _servo3.follow(_servo2);
        _servo1.follow(_servo3);
    }

    private void unfollow() {
        _servo2.follow(null);
        _servo3.follow(null);
        _servo1.follow(null);
    }
}
