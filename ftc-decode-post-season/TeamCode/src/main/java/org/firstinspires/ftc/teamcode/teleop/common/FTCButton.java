package org.firstinspires.ftc.teamcode.teleop.common;

import java.util.function.Supplier;

// Class for binding a binary button on an FTC Gamepad and for
// tracking its state.
public class FTCButton {

    public enum State { // button states
        TAP,        // button pressed
        DOUBLE_TAP, // pressed down in quick succession
        HELD,       // button pressed for more than one cycle
        UP,         // moment of release
        OFF,        // button not pressed for more than once cycle
    }

    private static final int DOUBLE_TAP_INTERVAL_MS = 250;
    private final Supplier<Boolean> buttonValue;
    private State state = State.OFF;
    private long lastTapped = -1;

    public FTCButton(Supplier<Boolean> pButtonValue) {
        buttonValue = pButtonValue;
    }

    // State.UP can be used to detect the release of a held button.
    public void update() {
        if (buttonValue.get()) {
            if (state == State.OFF || state == State.UP) {
                if (System.currentTimeMillis() - lastTapped < DOUBLE_TAP_INTERVAL_MS) {
                    lastTapped = System.currentTimeMillis();
                    state = State.DOUBLE_TAP;
                } else {
                    lastTapped = System.currentTimeMillis();
                    state = State.TAP;
                }
            } else {
                state = State.HELD;
            }
        } else if (state == State.HELD || state == State.TAP || state == State.DOUBLE_TAP)
            state = State.UP;
        else
            state = State.OFF;
    }

    public boolean is(State pState) {
        return state == pState;
    }

    public State getState() {
        return state;
    }

    public void reset() {
        state = State.OFF;
    }

}
