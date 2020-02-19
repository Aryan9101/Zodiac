package com.team1816.frc2020.subsystems;

import com.team1816.frc2020.Robot;
import com.team1816.frc2020.states.SuperstructureCommand;
import com.team1816.frc2020.states.SuperstructureState;
import com.team1816.frc2020.states.SuperstructureStateManager;
import com.team1816.frc2020.states.SuperstructureStateManager.WantedAction;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.Subsystem;
import com.team254.lib.vision.AimingParameters;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;


/**
 * The superstructure subsystem is the overarching class containing all components of the superstructure: the
 * turret, elevator, arm, and wrist. The superstructure subsystem also uses info from the vision system.
 * <p>
 * Instead of interacting individually with subsystems like the elevator and arm, the {@link Robot} class sends commands
 * to the superstructure, which individually decides how to move each subsystem to get there.
 * <p>
 * The Superstructure class also adjusts the overall goal based on the turret and elevator control modes.
 */
public class Superstructure extends Subsystem {

    private static Superstructure mInstance;
    private SuperstructureState state = new SuperstructureState();

    //TODO:

    private Shooter shooter = Shooter.getInstance();
    private Hopper hopper = Hopper.getInstance();
    private Turret turret = Turret.getInstance();

    private SuperstructureStateManager stateMachine = new SuperstructureStateManager();
    private WantedAction wantedAction = WantedAction.IDLE;

    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure() {
        super("superstructure");
    }

    public SuperstructureStateManager.SubsystemState getSuperStructureState() {
        return stateMachine.getSubsystemState();
    }

    public SuperstructureState getObservedState() {
        return state;
    }

    private synchronized void updateObservedState(SuperstructureState state) {
        state.shooterVelocity = shooter.getActualVelocity();
        state.turretAngle = turret.getTurretPositionDegrees();
        state.hopperIntake = hopper.getIntake(); //TODO: create method for checking hopper state
    }

    // Update subsystems from planner
    synchronized void setFromCommandState(SuperstructureCommand commandState) {

        shooter.setVelocity(commandState.shooterVelocity);
        turret.setTurretAngle(commandState.turretAngle);
        hopper.setIntake(commandState.hopperIntake); //TODO: how to manipulate hopper?
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            private SuperstructureCommand command;

            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateObservedState(state);

                    SmartDashboard.setDefaultString("Superstructure/wantedAction", wantedAction.toString());

                    command = stateMachine.update(timestamp, wantedAction, state);

                    setFromCommandState(command);
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    //TODO: Aiming parameters will involve Turret and Vision
    public synchronized Optional<AimingParameters> getLatestAimingParameters() {
        return mLatestAimingParameters;
    }

    public synchronized boolean isAtDesiredState() {
        return stateMachine.getSubsystemState() == SuperstructureStateManager.SubsystemState.WANTED_POSITION;
    }

    public void setWantedAction(WantedAction wantedAction) {
        this.wantedAction = wantedAction;
    }

   // TODO: Remove
    public synchronized void setCollectingMode() {
        System.err.println("Setting state to collecting mode!");
        setWantedAction(WantedAction.GO_TO_POSITION);
        stateMachine.setCollectorDown(true);
        stateMachine.setArmPosition(CargoShooter.ARM_POSITION_DOWN);
    }

    public synchronized void setRocketMode() {
        setWantedAction(WantedAction.GO_TO_POSITION);
        stateMachine.setArmPosition(CargoShooter.ARM_POSITION_MID);
        stateMachine.setCollectorDown(false);
    }

    public synchronized void setShootUpwardsMode() {
        System.out.println("Setting state to normal state!");
        setWantedAction(WantedAction.GO_TO_POSITION);
        stateMachine.setArmPosition(CargoShooter.ARM_POSITION_UP);
        stateMachine.setCollectorDown(false);
    }


    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
