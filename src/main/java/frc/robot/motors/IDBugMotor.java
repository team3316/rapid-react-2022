package frc.robot.motors;

public interface IDBugMotor {
    void setRotationFactors(PositionUnit positionUnit, 
                            VelocityUnit velocityUnit, 
                            double gearRatio, 
                            double wheelDiameterMeters, 
                            int upr);

    void setInverted(boolean inverted);

    void follow(IDBugMotor leader);

    void set(ControlMode mode, double value);

    void setupPIDF(PIDFGains gains);

    void setPosition(double value);

    double getVelocity();

    double getPosition();
}
