package com.team1816.frc2020.states;

import com.team1816.frc2020.subsystems.Shooter;
import com.team254.lib.util.Util;

public class SuperstructureState {
    public double shooterVelocity;
    public double turretAngle;
    public boolean elevatorIntake;
    public boolean spindexerIntake;
    public boolean collectorDeployed;
    public boolean wantHoming;

//    Turret:
//    maxPos (and degrees):
//    minPos (and degrees:

    public SuperstructureState(int shooterVelocity, double turretAngle, boolean elevatorIntake,
                               boolean spindexerIntake,
                               boolean collectorDeployed,
                               boolean wantHoming) {
        this.shooterVelocity = shooterVelocity;
        this.turretAngle = turretAngle;
        this.elevatorIntake = elevatorIntake;
        this.spindexerIntake = spindexerIntake;
        this.collectorDeployed = collectorDeployed;
        this.wantHoming = wantHoming;
    }

    public SuperstructureState() {
        this(0, 0, false, false, false, false);
    }

    public boolean inIllegalZone() {
        return !Shooter.getInstance().isVelocityNearTarget() && (elevatorIntake) && (spindexerIntake);
    }

    public boolean isInRange(SuperstructureState otherState, int shooterVelocityThreshold, double turretAngleThreshold) {
        return Util.epsilonEquals(otherState.shooterVelocity, shooterVelocity, shooterVelocityThreshold)
            && Util.epsilonEquals(otherState.turretAngle, turretAngle, turretAngleThreshold)
            && (otherState.elevatorIntake = elevatorIntake)
            && (otherState.spindexerIntake = spindexerIntake);
    }

    @Override
    public String toString() {
        return
            "SuperstructureState {"
                + "  shooterVelocity = " + shooterVelocity
                + "  turretPosition = " + turretAngle
                + "  elevatorIntake = " + elevatorIntake
                + "  spindexerIntake = " + spindexerIntake
                + "  collectorDeployed = " + collectorDeployed
                + "  wantHoming = " + wantHoming
                + " }";
    }
}
