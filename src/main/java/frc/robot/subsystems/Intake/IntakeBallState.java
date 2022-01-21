// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeBallState {
    public final boolean leftCargo;
    public final boolean rightCargo;
    public IntakeBallState(boolean leftCargo, boolean rightCargo){
        new Intake();
        this.leftCargo = leftCargo;
        this.rightCargo = rightCargo;
    }

    public boolean hasBoth(){
        return this.leftCargo && this.rightCargo;
    }
}
