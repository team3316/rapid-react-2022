package frc.robot.utils;

public class DoubleLatchedBoolean {
    private boolean mLast = false;

    public boolean update(boolean newValue) {
        boolean ret = newValue != mLast;
        mLast = newValue;
        return ret;
    }
}