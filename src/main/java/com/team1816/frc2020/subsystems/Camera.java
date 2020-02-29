package com.team1816.frc2020.subsystems;

import com.team1816.frc2020.RobotState;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.Subsystem;
import com.team254.lib.vision.TargetInfo;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera extends Subsystem {
    private static Camera INSTANCE;

    public static Camera getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Camera();
        }
        return INSTANCE;
    }

    // Components
    private final NetworkTable networkTable;

    // State
    private double deltaXAngle;
    private double distance;

    private double center_x;
    private double center_y;

    // Constants
    private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px

    private Camera() {
        super("camera");
        networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        networkTable.addEntryListener("center_x", (table, key, entry, value, flags) -> {
            if (value.getDouble() < 0) { return; }
            center_x = value.getDouble();

            //TODO: calculation should be done elsewhere (Turret?)
            var deltaXPixels = (value.getDouble() - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
            this.deltaXAngle = Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        networkTable.addEntryListener("center_y", (table, key, entry, value, flags) -> {
            if (value.getDouble() < 0) { return; }
            center_y = value.getDouble();

            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        networkTable.addEntryListener("distance", (table, key, entry, value, flags) -> {
            if (value.getDouble() < 0) { return; }
            this.distance = value.getDouble();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                RobotState.getInstance().resetVision();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Camera.this) {
                    RobotState.getInstance().addVisionUpdate(timestamp, new TargetInfo(center_x, center_y));
                }

            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };
        mEnabledLooper.register(mLoop);
    }

    public double getDeltaXAngle() {
        return deltaXAngle;
    }

    public double getDistance() {
        return distance;
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}
