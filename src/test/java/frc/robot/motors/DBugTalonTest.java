package frc.robot.motors;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.junit.BeforeClass;
import org.junit.Test;

import frc.robot.motors.units.UnitConversions;

public class DBugTalonTest {
    static DBugTalon _fx1;

    @BeforeClass
    public static void init() {
        DBugTalonTest._fx1 = new DBugTalon(1, TalonModel.TalonFX, new UnitConversions(2, 1, 1), FeedbackDevice.IntegratedSensor);
    }

    @Test
    public void set() {
        // DBugTalonTest._fx1.set(ControlMode.Velocity, 10, VelocityUnit.RPM);
        // DBugTalonTest._fx1.set(ControlMode.PercentOutput, 10);
        // assertEquals(10, DBugTalonTest._fx1.getMotorOutputPercent(), 0.0000001);
    }
}
