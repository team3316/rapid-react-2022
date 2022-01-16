package frc.robot.motors;

public class PIDFGains {
    public final double kP, kI, kD, kF, tolerance, iZone;
  
    public PIDFGains(double kP, double kI, double kD, double kF, double tolerance, double iZone) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kF = kF;
      this.tolerance = tolerance;
      this.iZone = iZone;
    }
  }
  