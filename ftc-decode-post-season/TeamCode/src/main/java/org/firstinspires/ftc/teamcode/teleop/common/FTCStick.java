package org.firstinspires.ftc.teamcode.teleop.common;

import java.util.function.Supplier;

// @GoodHousekeeping seal

public class FTCStick {
    public static final String TAG = FTCStick.class.getSimpleName();

    public enum StickAxisInversion {INVERT, DO_NOT_INVERT}

    public static final float STICK_ZERO_POSITION = 0.1F;

    public static final float CURVE_EXPONENT = 1;

    private final Supplier<Float> stickValue;
    private final StickAxisInversion invertAxis;

    public FTCStick(Supplier<Float> pStickValue, StickAxisInversion pInvertAxis) {
        stickValue = pStickValue;
        invertAxis = pInvertAxis;
    }

    public float getAdjustedValue() {
        float currentStickValue = stickValue.get();
        if (Math.abs(currentStickValue) < STICK_ZERO_POSITION){
            return 0;
        } else {
            float sign = Math.signum(currentStickValue) * ((invertAxis == StickAxisInversion.DO_NOT_INVERT) ? 1 :  -1);
            // rescale stick position from [STICK_ZERO_POSITION, 1] to [0, 1]
            // so there will not be a values jump at STICK_ZERO_POSITION
            float adjusted = (Math.abs(currentStickValue) - STICK_ZERO_POSITION) / (1 - STICK_ZERO_POSITION);
            // allow for non linear stick curve.
            // allows for stick to be less sensitive in the center, while maintaining full range of output.
            float curved = (float) Math.pow(adjusted, CURVE_EXPONENT);
            return curved * sign;
        }
    }

}