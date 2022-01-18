package frc.robot.motors;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PWM;

/**
* Standard hobby style servo.
*
* <p>The range parameters default to the appropriate values for the Hitec HS-322HD servo provided
* in the FIRST Kit of Parts in 2008.
*/
public class BetterServo extends PWM {
    protected final double kMaxServoAngle;
    protected final double kMinServoAngle;
    
    protected final double kDefaultMaxServoPWM = 2.4;
    protected final double kDefaultMinServoPWM = 0.6;
    
    /**
    * Constructor.<br>
    *
    * <p>By default {@value #kDefaultMaxServoPWM} ms is used as the maxPWM value<br>
    * By default {@value #kDefaultMinServoPWM} ms is used as the minPWM value<br>
    *
    * @param channel The PWM channel to which the servo is attached. 0-9 are on-board, 10-19 are on
    *     the MXP port
    */
    public BetterServo(final int channel) {
        this(channel, 200, 0);
        setBounds(kDefaultMaxServoPWM, 0, 0, 0, kDefaultMinServoPWM);
        setPeriodMultiplier(PeriodMultiplier.k4X);
        
        HAL.report(tResourceType.kResourceType_Servo, getChannel() + 1);
        SendableRegistry.setName(this, "Servo", getChannel());
    }

    /**
    * Constructor.<br>
    *
    *
    * @param channel The PWM channel to which the servo is attached. 0-9 are on-board, 10-19 are on
    *     the MXP port
    */
    /**
     * Constructor.<br>
     * 
     * @param channel The PWM channel to which the servo is attached. 0-9 are on-board, 10-19 are on
     *     the MXP port
     * @param maxAngle the maximum angle the servo can turn
     * @param minAngle the minimum angle the servo can turn
     */
    public BetterServo(final int channel, double maxAngle, double minAngle) {
        super(channel);

        this.kMaxServoAngle = maxAngle;
        this.kMinServoAngle = minAngle;
        setBounds(kDefaultMaxServoPWM, 0, 0, 0, kDefaultMinServoPWM);
        setPeriodMultiplier(PeriodMultiplier.k4X);
        
        HAL.report(tResourceType.kResourceType_Servo, getChannel() + 1);
        SendableRegistry.setName(this, "Servo", getChannel());
    }
    
    /**
    * Set the servo position.
    *
    * <p>Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
    *
    * @param value Position from 0.0 to 1.0.
    */
    public void set(double value) {
        setPosition(value);
    }
    
    /**
    * Get the servo position.
    *
    * <p>Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
    * This returns the commanded position, not the position that the servo is actually at, as the
    * servo does not report its own position.
    *
    * @return Position from 0.0 to 1.0.
    */
    public double get() {
        return getPosition();
    }
    
    /**
    * Set the servo angle.
    *
    * <p>The angles are based on the HS-322HD Servo, and have a range of 0 to 180 degrees.
    *
    * <p>Servo angles that are out of the supported range of the servo simply "saturate" in that
    * direction In other words, if the servo has a range of (X degrees to Y degrees) than angles of
    * less than X result in an angle of X being set and angles of more than Y degrees result in an
    * angle of Y being set.
    *
    * @param degrees The angle in degrees to set the servo.
    */
    public void setAngle(double degrees) {
        if (degrees < kMinServoAngle) {
            degrees = kMinServoAngle;
        } else if (degrees > kMaxServoAngle) {
            degrees = kMaxServoAngle;
        }
        
        setPosition(((degrees - kMinServoAngle)) / getServoAngleRange());
    }
    
    /**
    * Get the servo angle.
    *
    * <p>This returns the commanded angle, not the angle that the servo is actually at, as the servo
    * does not report its own angle.
    *
    * @return The angle in degrees to which the servo is set.
    */
    public double getAngle() {
        return getPosition() * getServoAngleRange() + kMinServoAngle;
    }
    
    private double getServoAngleRange() {
        return kMaxServoAngle - kMinServoAngle;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Servo");
        builder.addDoubleProperty("Value", this::get, this::set);
    }
}

