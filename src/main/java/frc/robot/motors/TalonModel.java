package frc.robot.motors;

public enum TalonModel {
    TalonFX,
    TalonSRX;

    public String toString() {
        switch (this) {
            case TalonFX:
                return "Talon FX";        
            case TalonSRX:
                return "Talon SRX";
        }
        throw new IllegalArgumentException("Unknown Talon type");
    }
}
