// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LED.RobotColorState;

public class LED extends SubsystemBase {
    private AddressableLED _led;
    private AddressableLEDBuffer _mainBuffer, _blinkBuffer;

    private boolean _isBlinking = false;

    private double _lastChange;
    private boolean _on;

    private final RobotColorState _onState;
    private final RobotColorState _offState;
    private final double _interval;

    public LED() {
        this._led = new AddressableLED(Constants.LED.port);
        this._mainBuffer = new AddressableLEDBuffer(Constants.LED.length);
        this._blinkBuffer = new AddressableLEDBuffer(Constants.LED.length);

        this._led.setLength(this._mainBuffer.getLength());

        this._led.setData(this._mainBuffer);
        this._led.start();

        this._lastChange = 0.0;
        this._interval = 0.0;

        this._onState = null;
        this._offState = null;

        this._on = false;
    }

    public void setLED(RobotColorState state) {
        for (int i = 0; i < this._mainBuffer.getLength(); i++) {
            this._mainBuffer.setLED(i, state.color);
        }

        this._led.setData(this._mainBuffer);
    }

    public void setBlink(RobotColorState state) {
        this._isBlinking = true;

        for(int i = 0; i < this._blinkBuffer.getLength(); i++){
            this._blinkBuffer.setLED(i, state.color);
        }

        this._led.setData(this._blinkBuffer);
    }

    @Override
    public void periodic() {
        if(this._isBlinking){
            double timestamp = Timer.getFPGATimestamp();
            if (timestamp - _lastChange > _interval){
                _on = !_on;
                _lastChange = timestamp;
                if (_on){
                    setLED(this._onState);
                } else {
                    setLED(this._offState);
                }
            } 
        }
    }
}
