package com.team1816.frc2020.states;

import com.team1816.frc2019.subsystems.CargoShooter;
import com.team254.lib.util.Util;

public class SuperstructureState {
    public int armPosition;
    public boolean isCollectorDown;

    public SuperstructureState(int armPosition, boolean isCollectorDown) {
        this.armPosition = armPosition;
        this.isCollectorDown = isCollectorDown;
    }

//    maxPos: 4027
//    midPos: 3230
//    minPos: 3015

    public SuperstructureState() {
        this(CargoShooter.ARM_POSITION_UP, false);
    }


    public boolean inIllegalZone(boolean allowSmallErrors) {
        return false;
    }

    public boolean isInRange(SuperstructureState otherState, int armPositionThreshold) {
        return Util.epsilonEquals(otherState.armPosition, armPosition, armPositionThreshold);
    }

    public boolean isInRange(SuperstructureState otherState) {
        return otherState.isCollectorDown == isCollectorDown;
    }



    @Override
    public String toString() {
        return
            "SuperstructureState {"
                + "  armPosition = " + armPosition
                + "  collectorDown = " + isCollectorDown
                + "}";
    }
}
