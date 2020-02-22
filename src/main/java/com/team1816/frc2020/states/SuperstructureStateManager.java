package com.team1816.frc2020.states;

import com.team1816.frc2020.subsystems.Collector;
import com.team1816.frc2020.subsystems.Hopper;
import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.frc2020.subsystems.Turret;
import com.team254.lib.util.Util;

public class SuperstructureStateManager {
    public enum WantedAction {
        IDLE,
        GO_TO_POSITION
    }
    
    public enum SubsystemState {
        WANTED_POSITION,
        MOVING_TO_POSITION,
        MANUAL
    }

    private SubsystemState systemState = SubsystemState.WANTED_POSITION;

    private SuperstructureMotionPlanner planner = new SuperstructureMotionPlanner();

    private SuperstructureState commandedState = new SuperstructureState();
    private SuperstructureState desiredEndState = new SuperstructureState();

    private double shooterVelocity = Shooter.getInstance().getActualVelocity();
    private double turretAngle = Turret.getInstance().getTurretPositionDegrees();
    private boolean elevatorIntake = Hopper.getInstance().getElevatorIntake();
    private boolean spindexerIntake = Hopper.getInstance().getSpindexerIntake();
    private boolean collectorDeployed = Collector.getInstance().isArmDown();
    private boolean wantHoming = false;

    public boolean scoringPositionChanged() {
        return !Util.epsilonEquals(desiredEndState.shooterVelocity, shooterVelocity, SuperstructureMotionPlanner.kShooterVelocityThreshold)
                || !Util.epsilonEquals(desiredEndState.turretAngle, turretAngle, SuperstructureMotionPlanner.kTurretAngleThreshold)
                || desiredEndState.collectorDeployed != collectorDeployed;
    }

    public synchronized SuperstructureState update(double timestamp, WantedAction wantedAction,
                                                     SuperstructureState currentState) {
        synchronized (SuperstructureStateManager.this) {
            SubsystemState newState;

            //TODO: Test simplification
            switch (systemState) {
                case WANTED_POSITION:
                    newState = handleDefaultTransitions(wantedAction, currentState);
                 //   System.out.println("State in IDLE:" + newState);
                    break;
                case MOVING_TO_POSITION:
                    newState = handleDefaultTransitions(wantedAction, currentState);
                 //   System.out.println("State in GO_TO_POSITION:" + newState);
                    break;

                case MANUAL:

                default:
                    System.out.println("major bruh alert: " + systemState);
                    newState = systemState;
                    break;
            }

            if (newState != systemState) {
                System.out.println(timestamp +": Superstructure changed state: " + systemState + " -> " + newState);
                systemState = newState;
            }

            commandedState = planner.update(currentState);

            return commandedState;
        }
    }

    private void updateMotionPlannerDesired(SuperstructureState currentState) {
        desiredEndState.shooterVelocity = shooterVelocity;
        desiredEndState.turretAngle = turretAngle;
        desiredEndState.elevatorIntake = elevatorIntake;
        desiredEndState.spindexerIntake = spindexerIntake;
        desiredEndState.collectorDeployed = collectorDeployed;
        desiredEndState.wantHoming = wantHoming;

        var desiredStateReturnValue = planner.setDesiredState(desiredEndState, currentState);
        System.out.println("SuperstructureStateManager::updateMotionPlannerDesired() -> desiredStateReturnValue: "
            + desiredStateReturnValue);
        // Push into elevator planner.
        if (!desiredStateReturnValue) {
            System.out.println("Unable to set cargo shooter/collector planner!");
        }

        System.out.println("Setting motion planner to " + desiredEndState.toString());

        shooterVelocity = desiredEndState.shooterVelocity;
        turretAngle = desiredEndState.turretAngle;
        elevatorIntake = desiredEndState.elevatorIntake;
        spindexerIntake = desiredEndState.spindexerIntake;
        collectorDeployed = desiredEndState.collectorDeployed;
    }

    private SubsystemState handleDefaultTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if (wantedAction == WantedAction.GO_TO_POSITION) {
            if (scoringPositionChanged()) {
                updateMotionPlannerDesired(currentState);
            } else if (planner.isFinished(currentState)) {
                return SubsystemState.WANTED_POSITION;
            }
            return SubsystemState.MOVING_TO_POSITION;
        } else {
            if (systemState == SubsystemState.MOVING_TO_POSITION && !planner.isFinished(currentState)) {
                return SubsystemState.MOVING_TO_POSITION;
            } else {
                return SubsystemState.WANTED_POSITION;
            }
        }
    }

    public SubsystemState getSubsystemState() {
        return systemState;
    }

    public void setShooterVelocity(double shooterVelocity) {
        this.shooterVelocity = shooterVelocity;
    }

    public void setTurretAngle(double turretAngle) {
        this.turretAngle = turretAngle;
    }

    public void setElevatorIntake(boolean elevatorIntake) {
        this.elevatorIntake = elevatorIntake;
    }

    public void setSpindexerIntake(boolean spindexerIntake) {
        this.spindexerIntake = spindexerIntake;
    }

    public void setCollectorDeployed(boolean collectorDeployed) {
        this.collectorDeployed = collectorDeployed;
    }

    public void setWantHoming(boolean wantHoming) {
        this.wantHoming = wantHoming;
    }
}



