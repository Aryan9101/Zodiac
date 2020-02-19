package com.team1816.frc2020.states;

import com.team1816.frc2020.subsystems.Shooter;
import com.team254.lib.util.Util;

public class SuperstructureState {
    public double shooterVelocity;
    public double turretAngle; //TODO: want degrees or ticks?
    public double hopperIntake;

    public boolean wantHoming;

//    Turret:
//    maxPos (and degrees):
//    minPos (and degrees:

    public SuperstructureState(int shooterVelocity, double turretAngle, double hopperIntake, boolean wantHoming) {
        this.shooterVelocity = shooterVelocity;
        this.turretAngle = turretAngle;
        this.hopperIntake = hopperIntake;
        this.wantHoming = wantHoming;
    }

    public SuperstructureState() {
        this(0, 0, 0, false);
    }

    public boolean inIllegalZone() {
        return !Shooter.getInstance().isVelocityNearTarget() && (hopperIntake > 0);
    }

    public boolean isInRange(SuperstructureState otherState, int shooterVelocityThreshold, int turretAngleThreshold) {
        return Util.epsilonEquals(otherState.shooterVelocity, shooterVelocity, shooterVelocityThreshold)
            && Util.epsilonEquals(otherState.turretAngle, turretAngle, turretAngleThreshold)
            && Util.epsilonEquals(otherState.hopperIntake, hopperIntake);
    }

    @Override
    public String toString() {
        return
            "SuperstructureState {"
                + "  shooterVelocity = " + shooterVelocity
                + "  turretPosition = " + turretAngle
                + "  hopperIntake = " + hopperIntake
                + " }";
    }
}
