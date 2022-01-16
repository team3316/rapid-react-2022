package frc.robot.motors;

public interface IDBugMotor {
    void setInverted(boolean inverted);

    void follow(IDBugMotor leader);

    void set(ControlMode mode, double value);

    void setupPIDF(PIDFGains gains);

    void setPosition(double value);

    double getVelocity(VelocityUnit unit);

    double getPosition(PositionUnit unit);
}
