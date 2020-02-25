package com.team1816.frc2020.states;

import com.team1816.frc2020.subsystems.Shooter;
import com.team254.lib.util.Util;

public class SuperstructureState {
    public double shooterVelocity;
    public double turretAngle;
    public double elevatorVelocity;
    public double spindexerVelocity;
    public boolean collectorDeployed;
    public boolean wantHoming;

//    Turret:
//    maxPos (and degrees):
//    minPos (and degrees:

    public SuperstructureState(int shooterVelocity, double turretAngle, double elevatorVelocity,
                               double spindexerIntake,
                               boolean collectorDeployed,
                               boolean wantHoming) {
        this.shooterVelocity = shooterVelocity;
        this.turretAngle = turretAngle;
        this.elevatorVelocity = elevatorVelocity;
        this.spindexerVelocity = spindexerIntake;
        this.collectorDeployed = collectorDeployed;
        this.wantHoming = wantHoming;
    }

    public SuperstructureState() {
        this(0, 0, 0, 0, false, false);
    }

    public boolean inIllegalZone() {
        return !Shooter.getInstance().isVelocityNearTarget() && (elevatorVelocity != 0) && (spindexerVelocity != 0);
    }

    public boolean isInRange(SuperstructureState otherState, int shooterVelocityThreshold, double turretAngleThreshold) {
        return Util.epsilonEquals(otherState.shooterVelocity, shooterVelocity, shooterVelocityThreshold)
            && Util.epsilonEquals(otherState.turretAngle, turretAngle, turretAngleThreshold)
            && Util.epsilonEquals(otherState.elevatorVelocity, elevatorVelocity)
            && Util.epsilonEquals(otherState.spindexerVelocity, spindexerVelocity);
    }

    @Override
    public String toString() {
        return
            "SuperstructureState {"
                + "  shooterVelocity = " + shooterVelocity
                + "  turretPosition = " + turretAngle
                + "  elevatorIntake = " + elevatorVelocity
                + "  spindexerVelocity = " + spindexerVelocity
                + "  collectorDeployed = " + collectorDeployed
                + "  wantHoming = " + wantHoming
                + " }";
    }
}
