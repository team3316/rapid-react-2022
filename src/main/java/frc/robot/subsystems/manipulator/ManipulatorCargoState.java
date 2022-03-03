// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

/** Add your docs here. */
public class ManipulatorCargoState {
    public final boolean leftCargo;
    public final boolean rightCargo;
    public ManipulatorCargoState(boolean leftCargo, boolean rightCargo){
        this.leftCargo = leftCargo;
        this.rightCargo = rightCargo;
    }

    public boolean hasBoth(){
        return leftCargo && rightCargo;
    }
}
