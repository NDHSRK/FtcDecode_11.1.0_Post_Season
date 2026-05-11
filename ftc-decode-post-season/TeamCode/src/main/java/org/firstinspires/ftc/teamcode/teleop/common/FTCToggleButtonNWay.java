package org.firstinspires.ftc.teamcode.teleop.common;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.function.Supplier;

// N-way toggle based on an EnumSet<E>.
//## According to https://www.baeldung.com/java-enumset --
// "The elements are stored following the order in which they are declared in the enum."
public class FTCToggleButtonNWay<E extends Enum<E>> extends FTCButton {

    //!! Don't use an array: see below private final E[] toggleValues;
    private final ArrayList<E> toggleArray = new ArrayList<>();
    private int toggleIndex = 0;

    public FTCToggleButtonNWay(Supplier<Boolean> pButtonValue, EnumSet<E> pToggleSet) {
        super(pButtonValue);
        //!! Don't use an array: illegal cast from Object[] to E[] toggleValues = (E[]) pToggleSet.toArray();
        toggleArray.addAll(pToggleSet);
    }

    public E toggle() {
        toggleIndex = (toggleIndex < (toggleArray.size() - 1)) ? ++toggleIndex : 0;
        return toggleArray.get(toggleIndex);
    }

    public E reverseToggle() {
        toggleIndex = (toggleIndex != 0 ? --toggleIndex : (toggleArray.size() - 1));
        return toggleArray.get(toggleIndex);
    }

    public E getToggleState() {
        return toggleArray.get(toggleIndex);
    }

    public void setToggleState(E pSetState) {
        toggleIndex = pSetState.ordinal();
    }
}
