package frc.robot.motors.units;

import frc.robot.motors.EncoderModel;

public class UnitConversions {
    public final double gearRatio;
    public final double wheelDiameterMeters;
    public final double upr;

    public UnitConversions(double gearRatio, double wheelDiameterMeters, EncoderModel model) {
        this(gearRatio, wheelDiameterMeters, model.upr);
    }

    public UnitConversions(double gearRatio, double wheelDiameterMeters, double upr) {
        this.gearRatio = gearRatio;
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.upr = upr;
    }

    public UnitConversions(double gearRatio, double wheelDiameterMeters) {
        this(gearRatio, wheelDiameterMeters, Integer.MAX_VALUE);
    }

    public UnitConversions(double gearRatio) {
        this(gearRatio, Integer.MAX_VALUE, Integer.MAX_VALUE);
    }

    public double getRotationsModifier(PositionUnit unit) {
        switch (unit) {
            case Rotations:
                return gearRatio;
            case Degrees:
                return gearRatio * 360;
            case Meters:
                if (wheelDiameterMeters == Integer.MAX_VALUE)
                    throw new UnsupportedOperationException("Cannot get distance without wheel diameter");
                return gearRatio * wheelDiameterMeters * Math.PI;
        }
        throw new IllegalArgumentException("Unknown PositionUnit");
    }

    public double getRPMModifier(VelocityUnit unit) {
        switch (unit) {
            case RPM:
                return gearRatio;
            case MetersPerSecond:
                if (wheelDiameterMeters == Integer.MAX_VALUE)
                    throw new UnsupportedOperationException("Cannot get distance without wheel diameter");
                return gearRatio * wheelDiameterMeters * Math.PI / 60;
        }
        throw new IllegalArgumentException("Unknown VelocityUnit");
    }
}
