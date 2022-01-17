package frc.robot.motors.units;

import frc.robot.motors.EncoderModel;

public class UnitConversions {
    public final double gearRatio;
    public final double wheelDiameterMeters;
    public final double upr;

    public UnitConversions(double gearRatio, double wheelDiameterMeters, EncoderModel model) {
        this.gearRatio = gearRatio;
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.upr = model.upr;
    }

    public UnitConversions(double gearRatio, double wheelDiameterMeters, double upr) {
        this.gearRatio = gearRatio;
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.upr = upr;
    }

    public double getRotationsModifier(PositionUnit unit) {
        switch (unit) {
            case Rotations:
                return gearRatio;
            case Degrees:
                return gearRatio * 360;
            case Meters:
                return gearRatio * wheelDiameterMeters * Math.PI;
        }
        throw new IllegalArgumentException("Unknown PositionUnit");
    }

    public double getRPMModifier(VelocityUnit unit) {
        switch (unit) {
            case RPM:
                return gearRatio;
            case MetersPerSecond:
                return gearRatio *  wheelDiameterMeters * Math.PI / 60;
        }
        throw new IllegalArgumentException("Unknown VelocityUnit");
    }
}
