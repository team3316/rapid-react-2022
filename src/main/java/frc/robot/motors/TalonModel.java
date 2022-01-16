package frc.robot.motors;

public enum TalonModel {
    TalonFX("Talon FX"),
    TalonSRX("Talon SRX");

    public final String name;

    private TalonModel(String name) {
        this.name = name;
    }
}
