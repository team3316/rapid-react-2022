package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon extends SubsystemBase{
    /**
     *
     */
    private static final double G_METERS_PER_SECOND_SQUARED = 9.81;
    private TalonSRX _pigeonTalon;
    private PigeonIMU _pigeon;

    public Pigeon() {
        _pigeonTalon = new TalonSRX(0);
        _pigeon = new PigeonIMU(_pigeonTalon);
    }

    public double[] getAcceleration() {
        short[] rawValues = new short[3];
        _pigeon.getBiasedAccelerometer(rawValues);
        
        double[] values = new double[3];
        for (int i = 0; i < values.length; i++) {
            values[i] = ((double) rawValues[i]) / (1 << 14) * G_METERS_PER_SECOND_SQUARED;
        }

        return values;
    }
}
