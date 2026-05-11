package org.firstinspires.ftc.teamcode.teleop.common;

import java.util.function.Supplier;

// @GoodHousekeeping seal
public class FTCTrigger {
    public static final String TAG = FTCTrigger.class.getSimpleName();

    public static final float TRIGGER_ZERO_POSITION = 0.1f;

    public static final float CURVE_EXPONENT = 1;

    private final Supplier<Float> triggerValue;

    public FTCTrigger(Supplier<Float> pTriggerValue) {
        triggerValue = pTriggerValue;
    }

    public float getTriggerValue() {
        float currentTriggerValue = triggerValue.get();

        if (Math.abs(currentTriggerValue) < TRIGGER_ZERO_POSITION){
            return 0;
        } else {
            float sign = Math.signum(currentTriggerValue);
            // rescale trigger position from [TRIGGER_ZERO_POSITION, 1] to [0, 1]
            // so there will not be a values jump at TRIGGER_ZERO_POSITION
            float adjusted = (Math.abs(currentTriggerValue) - TRIGGER_ZERO_POSITION) / (1 - TRIGGER_ZERO_POSITION);
            // allow for non linear trigger curve.
            // allows for trigger to be less sensitive when lightly pulled, while maintaining full range of output.
            float curved = (float) Math.pow(adjusted, CURVE_EXPONENT);
            return curved * sign;
        }
    }

}
