package frc.robot.autonomous;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.plans.TestPlan;
import frc.robot.subsystems.drivetrain.Drivetrain;


public class AutoSelector {
    private final static Map<String, Command> map = new HashMap<>();
    private static boolean _initialized = false;

    public static void initialize(Drivetrain drivetrain) {
        map.put("Test Auto", new TestPlan(drivetrain));

        _initialized = true;
    }


    public static Command get(String key) {
        if (!_initialized) {
            throw new IllegalStateException("AutoSelector.initialize() must be called before this method");
        }
        Command val = map.get(key);
        if(val == null) {
            throw new IllegalArgumentException("Unknown command");
        }
        return val;
    }

}
