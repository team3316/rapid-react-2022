package frc.robot.motors;

public enum EncoderModel {
    CTREMagEncoder(4096),
    CANCoder(4096),
    Bourns(1024),
    FalconInternal(2048),
    NeoInternal(3316); // There is no api that uses this number

    public final int upr;

    private EncoderModel(int upr) {
        this.upr = upr;
    }


    
}
