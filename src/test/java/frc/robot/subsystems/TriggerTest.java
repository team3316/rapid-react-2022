package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.BeforeClass;
import org.junit.Test;

import frc.robot.subsystems.Trigger.Side;
import frc.robot.subsystems.Trigger.TriggerState;

public class TriggerTest {
    static Trigger trigger;

    @BeforeClass
    public static void init() {
        TriggerTest.trigger = new Trigger();
    }

    @Test
    public void setStateTest() {
        trigger.setState(TriggerState.IN, Side.LEFT);
        assertEquals(TriggerState.IN, trigger.getState(Side.LEFT));

        trigger.setState(TriggerState.OUT, Side.LEFT);
        assertEquals(TriggerState.OUT, trigger.getState(Side.LEFT));

        trigger.setState(TriggerState.IN, Side.RIGHT);
        assertEquals(TriggerState.IN, trigger.getState(Side.RIGHT));

        trigger.setState(TriggerState.OUT, Side.RIGHT);
        assertEquals(TriggerState.OUT, trigger.getState(Side.RIGHT));
    }
}
