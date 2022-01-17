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

    public double rotationsApplyModifier(double value, PositionUnit unit) {
        switch (unit) {
            case Rotations:
                return value * gearRatio;
            case Degrees:
                return value * gearRatio * 360;
            case Meters:
                return value * gearRatio * wheelDiameterMeters * Math.PI;
        }
        throw new IllegalArgumentException("Unknown PositionUnit");
    }

    public double rpmApplyModifier(double value, VelocityUnit unit) {
        switch (unit) {
            case RPM:
                return value * gearRatio;
            case MetersPerSecond:
                return value * gearRatio *  wheelDiameterMeters * Math.PI / 60;
        }
        throw new IllegalArgumentException("Unknown VelocityUnit");
    }
}
