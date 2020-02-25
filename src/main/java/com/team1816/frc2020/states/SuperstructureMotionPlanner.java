package com.team1816.frc2020.states;

import com.team1816.frc2020.subsystems.Shooter;
import com.team1816.frc2020.subsystems.Turret;
import edu.wpi.first.wpilibj.Timer;

import java.time.Instant;
import java.util.LinkedList;
import java.util.Optional;

public class SuperstructureMotionPlanner {

    // Constants
    public static final int kShooterVelocityThreshold = Shooter.VELOCITY_THRESHOLD;
    public static final double kTurretAngleThreshold = 0.5;

    static class SubCommand {

        protected boolean commandedCollectorDown;
        protected double commandedShooterSpeed;
        protected double commandedSpindexerSpeed;
        protected double commandedElevatorSpeed;

        public SubCommand(SuperstructureState endState) {
            mEndState = endState;
        }

        public SuperstructureState mEndState;

        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, kShooterVelocityThreshold, kTurretAngleThreshold);
        }

        public void update() { }
    }

    static class WaitForHomeSubCommand extends SubCommand {
        public WaitForHomeSubCommand(SuperstructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return super.isFinished(currentState) && Turret.getInstance().getPositionError() < 10;
        }
    }

    //TODO: Delete zeta specific subcommands
    static class WaitForCollectorSubCommand extends SubCommand {
        public WaitForCollectorSubCommand(SuperstructureState endState, boolean isCollectorDown) {
            super(endState);
            commandedCollectorDown = isCollectorDown;
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return super.isFinished(currentState);
        }

        @Override
        public void update() {
            mEndState.collectorDeployed = commandedCollectorDown;
        }
    }

    static class WaitForShooterSubCommand extends SubCommand {

        public WaitForShooterSubCommand(SuperstructureState endState, double shooterSpeed) {
            super(endState);
            commandedShooterSpeed = shooterSpeed;
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return super.isFinished(currentState);
        }

        @Override
        public void update() {
            mEndState.shooterVelocity = commandedShooterSpeed;
        }
    }

    static class WaitForElevatorSubCommand extends SubCommand {

        public WaitForElevatorSubCommand(SuperstructureState endState, double elevatorSpeed) {
            super(endState);
            commandedElevatorSpeed = elevatorSpeed;
        }

        @Override
        public void update() {
            mEndState.elevatorVelocity = commandedElevatorSpeed;
        }
    }

    static class InstantSpindexer extends SubCommand {

        public InstantSpindexer(SuperstructureState endState, double spindexerVelocity) {
            super(endState);
            commandedSpindexerSpeed = spindexerVelocity;
        }
        @Override
        public void update() {
            mEndState.spindexerVelocity = commandedSpindexerSpeed;
        }
    }

    static class WaitForTime extends SubCommand {
        boolean started;
        double startTime;
        double waitTime;
        double elapsedTime;

        public WaitForTime(SuperstructureState endState, double waitTime) {
            super(endState);
            this.waitTime = waitTime;
        //    System.out.println("WaitForTime INITIALIZED with Wait Time: " + this.waitTime);
        }

        @Override
        public void update() {
            super.update();
            if (!started) {
                startTime = Timer.getFPGATimestamp();
                started = true;
            }
            elapsedTime = Timer.getFPGATimestamp() - startTime;
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
          //  System.out.println("isFinished of WaitForTimeSubCommand called");

          //  System.out.println("WaitForTime::elapsedTime: " + elapsedTime);
            return elapsedTime > waitTime && super.isFinished(currentState);
        }
    }

    protected SuperstructureState mCommandedState = new SuperstructureState();
    protected SuperstructureState mIntermediateCommandState = new SuperstructureState();
    protected LinkedList<SubCommand> mCommandQueue = new LinkedList<>();
    protected Optional<SubCommand> mCurrentCommand = Optional.empty();


    public synchronized boolean setDesiredState(SuperstructureState desiredState, SuperstructureState currentState) {

        // Limit illegal inputs.
//        desiredState.armPosition = Util.limit(desiredState.armPosition,
//                CargoShooter.ARM_POSITION_UP, CargoShooter.ARM_POSITION_DOWN);

        if (desiredState.inIllegalZone()) {
            System.err.println("Desired State in Illegal Zone!");
            return false;
        }

        mCommandQueue.clear();

        if (desiredState.shooterVelocity > 0 && desiredState.wantHoming) {
            System.out.println("setDesiredState IF want to shoot");
            mCommandQueue.add(new InstantSpindexer(mIntermediateCommandState, desiredState.spindexerVelocity));
            mCommandQueue.add(new WaitForHomeSubCommand(mIntermediateCommandState));
            mCommandQueue.add(new WaitForShooterSubCommand(mIntermediateCommandState, desiredState.shooterVelocity));
            mCommandQueue.add(new WaitForElevatorSubCommand(mIntermediateCommandState, desiredState.elevatorVelocity));
            System.out.println("Queuing WaitForCollectingSubCommand - shooter will shoot");
        } else if (desiredState.collectorDeployed && desiredState.spindexerVelocity > 0) {
            System.out.println("setDesiredState IF want to collect");
            mCommandQueue.add(new InstantSpindexer(mIntermediateCommandState, -1));
            mCommandQueue.add(new WaitForCollectorSubCommand(mIntermediateCommandState, desiredState.collectorDeployed));
            mCommandQueue.add(new InstantSpindexer(mIntermediateCommandState, 1));
        } else {
            System.out.println("resetting robot");
            mCommandQueue.add(new WaitForCollectorSubCommand(mIntermediateCommandState, false));
            mCommandQueue.add(new WaitForShooterSubCommand(mIntermediateCommandState, 0));
            mCommandQueue.add(new InstantSpindexer(mIntermediateCommandState, 0));
            mCommandQueue.add(new WaitForElevatorSubCommand(mIntermediateCommandState, 0));
        }

         mCurrentCommand = Optional.empty();

        return true;
    }

    public void reset(SuperstructureState currentState) {
        mIntermediateCommandState = currentState;
        mCommandQueue.clear();
        mCurrentCommand = Optional.empty();
    }

    public boolean isFinished(SuperstructureState currentState) {
        return mCurrentCommand.isPresent() && mCommandQueue.isEmpty();
    }

    public SuperstructureState update(SuperstructureState currentState) {
        if (mCurrentCommand.isEmpty() && !mCommandQueue.isEmpty()) {
            mCurrentCommand = Optional.of(mCommandQueue.remove());
        }

        if (mCurrentCommand.isPresent()) {
            SubCommand subCommand = mCurrentCommand.get();
            subCommand.update();
            mIntermediateCommandState = subCommand.mEndState;
            //TODO: how to check finished logic?
            boolean finished = subCommand.isFinished(currentState);
            System.out.println(mCurrentCommand + "finished? :" + finished);
            if (finished && !mCommandQueue.isEmpty()) {
                // Let the current command persist until there is something in the queue. or not. desired outcome
                // unclear.
                mCurrentCommand = Optional.empty();
            }
        }

//        mCommandedState.armPosition =
//            Util.limit(mIntermediateCommandState.armPosition,
//                CargoShooter.ARM_POSITION_UP, CargoShooter.ARM_POSITION_DOWN);
//        mCommandedState.isCollectorDown = mIntermediateCommandState.isCollectorDown;

        return mCommandedState;
    }
}
