package com.team1816.lib.hardware;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;

public class MotorUtil {
    /**
     * checks the specified error code for issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + errorCode, false);
        }
    }

    public static void configCurrentLimit(
        IMotorControllerEnhanced motor,
        boolean enabled,
        double continuousCurrentLimit,
        double peakCurrentLimit,
        double peakThresholdSeconds
    ) {
        var limitConfig = new SupplyCurrentLimitConfiguration(enabled, continuousCurrentLimit, peakCurrentLimit, peakThresholdSeconds);
        if (motor instanceof TalonFX) {
            checkError(
                ((TalonFX) motor).configSupplyCurrentLimit(limitConfig),
                "Unable to configure motor ID" + motor.getDeviceID() + " current limit"
            );
        } else if (motor instanceof TalonSRX) {
            checkError(
                ((TalonSRX) motor).configSupplyCurrentLimit(limitConfig),
                "Unable to configure motor ID" + motor.getDeviceID() + " current limit"
            );
        }
    }

    public static double getSupplyCurrent(IMotorControllerEnhanced motor) {
        if (motor instanceof TalonFX) {
            return ((TalonFX) motor).getSupplyCurrent();
        } else if (motor instanceof TalonSRX) {
            return ((TalonSRX) motor).getSupplyCurrent();
        }
        return -1;
    }
}
