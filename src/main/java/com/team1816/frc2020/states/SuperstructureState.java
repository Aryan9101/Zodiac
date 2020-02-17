package com.team1816.frc2020.states;

import com.team254.lib.util.Util;

public class SuperstructureState {
    public double shooterVelocity;
    public double turretPosition;
    public boolean hopperDeployed;

//    Turret:
//    maxPos (and degrees):
//    minPos (and degrees:

    public SuperstructureState(int shooterVelocity, double turretPosition, boolean hopperDeployed) {
        this.shooterVelocity = shooterVelocity;
        this.turretPosition = turretPosition;
        this.hopperDeployed = hopperDeployed;
    }

    public SuperstructureState() {
        this(/*TODO: update*/0, /*TODO: update*/0, false);
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
                + "  shooterVelocity = " + shooterVelocity
                + "  turretPosition = " + turretPosition
                + "  hopperDeployed = " + hopperDeployed
                + " }";
    }
}
