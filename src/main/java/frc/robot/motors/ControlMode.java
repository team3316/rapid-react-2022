package frc.robot.motors;

public enum ControlMode {
    /**
	 * Percent output [-1,1]
	 */
	PercentOutput,
	/**
	 * Position closed loop
	 */
	Position,
	/**
	 * Velocity closed loop
	 */
	Velocity,
	/**
	 * Input current closed loop
	 */
	Current
}
