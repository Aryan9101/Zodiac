package com.team1816.frc2020.states;

import com.team1816.frc2020.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;
import java.util.Optional;

public class SuperstructureMotionPlanner {
    protected static boolean isFinished;

    // Constants
    public static final int kShooterVelocityThreshold = Shooter.VELOCITY_THRESHOLD;
    public static final double kTurretAngleThreshold = 0.5;

    static class SubCommand {

        public SubCommand(SuperstructureState endState) {
            mEndState = endState;
        }

        public SuperstructureState mEndState;

        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, kShooterVelocityThreshold, kTurretAngleThreshold);
        }

        @Deprecated
        public boolean isFinished() {
            return isFinished;
        }

        public void update() { }
    }

    static class WaitForShootSubCommand extends SubCommand {
        public WaitForShootSubCommand(SuperstructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished(SuperstructureState currentState, int shooterVelocityThreshold, int turretPositionThreshold) {
            return super.isFinished(currentState, shooterVelocityThreshold, turretPositionThreshold);
        }
    }

    static class WaitForHomeSubCommand extends SubCommand {
        public WaitForHomeSubCommand(SuperstructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished() {
            return super.isFinished();
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
            mEndState.isCollectorDown = commandedCollectorDown;
        }
    }

    static class WaitForShooterSubCommand extends SubCommand {

        public WaitForShooterSubCommand(SuperstructureState endState, int armPosition) {
            super(endState);
            commandedArmPosition = armPosition;
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return super.isFinished(currentState, 30);
        }

        @Override
        public void update() {
            mEndState.armPosition = commandedArmPosition;
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

        if () {
            System.out.println("setDesiredState IF");
            // Target or current below mid position - arm will be moving through collector box
            // Ensure collector down
            mCommandQueue.add(new WaitForCollectorSubCommand(mIntermediateCommandState, desiredState.isCollectorDown));
            mCommandQueue.add(new WaitForTime(mIntermediateCommandState,0.15));
            mCommandQueue.add(new WaitForShooterSubCommand(mIntermediateCommandState, desiredState.armPosition));
            System.out.println("Queuing WaitForCollectingSubCommand - arm will be moving through collector box");
        } else {
            System.out.println("setDesiredState ELSE");
            // Lift collector if target position above or equal to mid position
            mCommandQueue.add(new WaitForShooterSubCommand(mIntermediateCommandState, desiredState.armPosition));
            mCommandQueue.add(new WaitForTime(mIntermediateCommandState,0.15));
            System.out.println("Queuing WaitForENDCollectingSubCommand - arm will be above collector box");
            mCommandQueue.add(new WaitForCollectorSubCommand(mIntermediateCommandState, desiredState.isCollectorDown));
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
