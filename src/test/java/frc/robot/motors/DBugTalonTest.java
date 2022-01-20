package frc.robot.motors;

import static org.junit.Assert.assertEquals;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import org.junit.BeforeClass;
import org.junit.Test;

import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

public class DBugTalonTest {
    static DBugTalon _fx1;

    @BeforeClass
    public static void init() {
        _fx1 = new DBugTalon(1, TalonModel.TalonFX, new UnitConversions(2, 1, 1), FeedbackDevice.IntegratedSensor);
    }

    @Test
    public void set() {
        // _fx1.set(ControlMode.Velocity, 10, VelocityUnit.RPM);
        // _fx1.set(ControlMode.PercentOutput, 10);
        // assertEquals(10, _fx1.getMotorOutputPercent(), 0.0000001);
    }
}
