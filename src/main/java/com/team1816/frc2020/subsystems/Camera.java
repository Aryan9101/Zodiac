package com.team1816.frc2020.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableEntryListener;

import java.util.Objects;

public class Camera {
    private static Camera INSTANCE;

    public static Camera getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Camera();
        }
        return INSTANCE;
    }

    private Pose pose;

    private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px
    public static final double VISION_HOMING_BIAS = 0 /* 1.75 */; // deg

    private Camera() {
        var networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        networkTable.addEntryListener((table, key, entry, value, flags) -> {
            if (value.getDouble() < 0) { return; }
            var deltaXPixels = (value.getDouble() - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
            // var deltaXAngle = deltaXPixels * (CAMERA_FOV / VIDEO_WIDTH) + VISION_HOMING_BIAS; // Multiply by FOV to pixel ratio
            // TODO: test this formula
            var deltaXAngle = Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH) + VISION_HOMING_BIAS);
            pose.setDeltaXAngle(deltaXAngle);
            pose.setDistance(table.getEntry("distance").getDouble(-1));
        }, TableEntryListener.kNew | TableEntryListener.kUpdate);
    }

    public Pose getPose() {
        return pose;
    }

    public static class Pose {
        private double deltaXAngle;
        private double distance;

        public Pose(double deltaXAngle, double distance) {
            this.deltaXAngle = deltaXAngle;
            this.distance = distance;
        }

        public void setDeltaXAngle(double deltaXAngle) {
            this.deltaXAngle = deltaXAngle;
        }

        public void setDistance(double distance) {
            this.distance = distance;
        }

        public double getDeltaXAngle() {
            return deltaXAngle;
        }

        public double getDistance() {
            return distance;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            Pose pose = (Pose) o;
            return Double.compare(pose.deltaXAngle, deltaXAngle) == 0 &&
                Double.compare(pose.distance, distance) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(deltaXAngle, distance);
        }
    }
}
