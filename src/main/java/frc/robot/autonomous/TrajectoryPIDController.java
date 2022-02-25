package frc.robot.autonomous;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.motors.PIDFGains;

public class TrajectoryPIDController extends ProfiledPIDController {
    // Hijacking the ProfiledPIDController interface and doing our own thing.

    private final static int TAPS = 3;
    private PIDFGains m_gains;
    private LinearFilter m_robotPosition; // Use moving average to clean measurements
    private double m_lastGoal; // Assuming goal is smooth, so no moving average needed.
    private double m_lastVelocityError;
    public double kPContribution, kIContribution, kDContribution, kFContribution = 0.0;
    
    public TrajectoryPIDController(PIDFGains gains) {
        super(0, 0, 0, new Constraints(100, 100));
        m_gains = gains;
    }

    @Override
    public double calculate(double measurement, double goal) {
        double goalVelocity = (goal - m_lastGoal) / getPeriod();
        double positionError = goal - m_robotPosition.calculate(measurement);
        double velocityError = positionError / (getPeriod() * TAPS / 2);
        double errorAcceleration = (velocityError - m_lastVelocityError) / getPeriod();

        m_lastGoal = goal;
        m_lastVelocityError = velocityError;

        kPContribution = m_gains.kP * velocityError;
        kIContribution = m_gains.kI * positionError;
        kDContribution = m_gains.kD * errorAcceleration;
        kFContribution = m_gains.kF * goalVelocity;

        return kPContribution + kIContribution + kDContribution + kFContribution;
    }

    @Override
    public void reset(double measuredPosition) {
        super.reset(measuredPosition);

        m_robotPosition = LinearFilter.movingAverage(TAPS);
        m_robotPosition.calculate(measuredPosition);

        m_lastGoal = measuredPosition;
        m_lastVelocityError = 0;
    }
}