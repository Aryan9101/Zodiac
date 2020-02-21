package com.team1816.frc2020.states;

import com.team1816.frc2020.subsystems.Collector;
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

    private SuperstructureCommand command = new SuperstructureCommand();
    private SuperstructureState commandedState = new SuperstructureState();
    private SuperstructureState desiredEndState = new SuperstructureState();

    private double shooterVelocity = Shooter.getInstance().getActualVelocity();
    private double turretAngle = Turret.getInstance().getTurretPositionTicks(); //TODO: @SuperstructureState: want degrees?
    private boolean collectorDeployed = Collector.getInstance().isArmDown();


    private boolean hopperDeployed;

    public boolean scoringPositionChanged() {
        var scoringPositionChanged = !Util.epsilonEquals(desiredEndState.shooterVelocity, shooterVelocity, 3000) ||
            !Util.epsilonEquals(desiredEndState.turretAngle, turretAngle, /* degrees or ticks? */ 10)
            || desiredEndState.collectorDeployed != collectorDeployed;

        return scoringPositionChanged;
    }

    public synchronized SuperstructureCommand update(double timestamp, WantedAction wantedAction,
                                                     SuperstructureState currentState) {
        synchronized (SuperstructureStateManager.this) {
            SubsystemState newState;

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
            command.armPosition = Util.limit(commandedState.armPosition,
                CargoShooter.ARM_POSITION_UP, CargoShooter.ARM_POSITION_DOWN);
            command.collectorDown = commandedState.isCollectorDown;

            return command;
        }
    }

    private void updateMotionPlannerDesired(SuperstructureState currentState) {

        desiredEndState.shooterVelocity = shooterVelocity;
        desiredEndState.turretAngle = turretAngle;

        var desiredStateReturnValue = planner.setDesiredState(desiredEndState, currentState);
        System.out.println("SuperstructureStateManager::updateMotionPlannerDesired() -> desiredStateReturnValue: "
            + desiredStateReturnValue);
        // Push into elevator planner.
        if (!desiredStateReturnValue) {
            System.out.println("Unable to set cargo shooter/collector planner!");
        }

        System.out.println("Setting motion planner to armPosition: " + desiredEndState.armPosition
            + " || collectorDown: " + desiredEndState.isCollectorDown);

        armPosition = desiredEndState.armPosition;
        isCollectorDown = desiredEndState.isCollectorDown;

        System.out.println("Collector state: " + isCollectorDown);
    }

    public synchronized void setArmPosition(int armPosition) {
        this.armPosition = armPosition;
    }

    public int getArmPosition() {
        return armPosition;
    }

    public synchronized void setCollectorDown(boolean collectorDown) {
        isCollectorDown = collectorDown;
    }

    public boolean isCollectorDown() {
        return isCollector;
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
}



