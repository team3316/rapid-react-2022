// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
    private AddressableLED _led;
    private AddressableLEDBuffer _buffer;
    private Blinker _blinker;

    private class Blinker {
        private double _lastChange;
        private boolean _on;

        private final Color _onColor;
        private final Color _offColor;
        private final double _interval;
        

        public Blinker(double interval, Color onColor, Color offColor) {
            _interval = interval;
            _onColor = onColor;
            _offColor = offColor;
        }

        public void update() {
            double timestamp = Timer.getFPGATimestamp();
            if (timestamp - _lastChange > _interval){
                _on = !_on;
                _lastChange = timestamp;
                if (_on){
                    setLED(_onColor);
                } else {
                    setLED(_offColor);
                }
            } 
        }
        
    }

    public LED() {
        this._led = new AddressableLED(Constants.LED.port);
        this._buffer = new AddressableLEDBuffer(Constants.LED.length);

        this._led.setLength(this._buffer.getLength());

        this._led.setData(this._buffer);
        this._led.start();
    }

    private void setLED(Color color) {
        _blinker = null;
        for (int i = 0; i < this._buffer.getLength(); i++) {
            this._buffer.setLED(i, color);
        }

        this._led.setData(this._buffer);
    }

    public void setBlink(RobotColorState state) {

        _blinker = new Blinker(1, state.color, new Color(0,0,0));
        this._led.stop();
    }

    public enum RobotColorState {
        ONE_CARGO(Constants.LED.oneCargoColor), // collect one CARGO
        TWO_CARGO(Constants.LED.bothCargoColor), // collect two CARGO
        COLLECT(Constants.LED.noneCargoColor), // defualt color for collecting (if there aren't CARGOs)
        ARM_UP(Constants.LED.armUpColor), // arm in shooting position
        FIFTEEN_SEC(Constants.LED.fifteenSecColor), // fifteen seconds left until the match ends
        FIVE_SEC(Constants.LED.fiveSecColor), // five seconds left until the match ends
        DEFAULT(Constants.LED.defaultCargoColor); // defualt color

        public Color color;

        private RobotColorState(Color color) {
            this.color = color;
        }
    }

    public void setRobotLEDs(RobotColorState state) {
        setLED(state.color);
    }

    @Override
    public void periodic() {
        if(_blinker != null) 
            _blinker.update();
    }
}
