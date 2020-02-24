package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.CANifier;
import com.team1816.frc2020.Robot;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class LedManager extends Subsystem {
    public static final String NAME = "ledmanager";

    private static LedManager INSTANCE;

    // Components
    private final CANifier canifier;
    private final AddressableLED ledStrip;

    // State
    private boolean blinkMode;
    private boolean blinkLedOn = false;
    private boolean outputsChanged = true;
    private AddressableLEDBuffer ledBuffer;

    private int ledR;
    private int ledG;
    private int ledB;

    private int period; // ms
    private long lastWriteTime = System.currentTimeMillis();

    // Constants
    private static final int LED_PWM_PORT = ((int) factory.getConstant(NAME, "ledPort"));
    private static final int LED_STRIP_LENGTH = ((int) factory.getConstant(NAME, "stripLength"));

    private LedManager() {
        super(NAME);
        this.canifier = factory.getCanifier(NAME);
        this.ledR = 0;
        this.ledG = 0;
        this.ledB = 0;
        this.ledStrip = new AddressableLED(LED_PWM_PORT);
        this.ledBuffer = new AddressableLEDBuffer(LED_STRIP_LENGTH);

        // Expensive operation, set only once.
        ledStrip.setLength(ledBuffer.getLength());

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public static LedManager getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LedManager();
        }
        return INSTANCE;
    }

    public void setLedColor(int r, int g, int b) {
        if (this.ledR != r || this.ledG != g || this.ledB != b) {
            this.ledR = r;
            this.ledG = g;
            this.ledB = b;
            outputsChanged = true;
        }
    }

    /**
     * @param r      LED color red value (0-255)
     * @param g      LED color green value (0-255)
     * @param b      LED color blue value (0-255)
     * @param period milliseconds
     */
    public void setLedColorBlink(int r, int g, int b, int period) {
        // Period is in milliseconds
        setLedColor(r, g, b);
        blinkMode = true;
        this.period = period;
        outputsChanged = true;
    }

    public void setLedColorBlink(int r, int g, int b) {
        // Default period of 1 second
        setLedColorBlink(r, g, b, 1000);
    }

    public void indicateStatus(RobotStatus status) {
        blinkMode = false;
        outputsChanged = true;
        setLedColor(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void blinkStatus(RobotStatus status) {
        setLedColorBlink(status.getRed(), status.getGreen(), status.getBlue());
    }

    public int[] getLedColor() {
        return new int[]{ledR, ledG, ledB};
    }

    public boolean isBlinkMode() {
        return blinkMode;
    }

    public double getPeriod() {
        return period;
    }

    private void writeLedHardware(int r, int g, int b) {
        canifier.setLEDOutput(r / 255.0, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(g / 255.0, CANifier.LEDChannel.LEDChannelA);
        canifier.setLEDOutput(b / 255.0, CANifier.LEDChannel.LEDChannelC);
    }

    private void writeEntireBuffer(int r, int g, int b) {
        writeBufferRegion(0, ledBuffer.getLength(), r, g, b);
    }

    private void writeBufferRegion(int start, int end, int r, int g, int b) {
        for (int i = start; i < end; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        ledStrip.setData(ledBuffer);
    }

    @Override
    public void writePeriodicOutputs() {
        if (canifier != null) {
            if (blinkMode) {
                if (System.currentTimeMillis() >= lastWriteTime + (period / 2)) {
                    if (blinkLedOn) {
                        writeLedHardware(0, 0, 0);
                        blinkLedOn = false;
                    } else {
                        writeLedHardware(ledR, ledG, ledB);
                        blinkLedOn = true;
                    }
                    lastWriteTime = System.currentTimeMillis();
                }
            } else if (outputsChanged) {
                System.out.printf("R: %d, G: %d, B: %d%n", ledR, ledG, ledB);
                writeLedHardware(ledR, ledG, ledB);
                outputsChanged = false;
            }
        }
    }

    @Override
    public void stop() {

    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        super.registerEnabledLoops(mEnabledLooper);
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                LedManager.this.writePeriodicOutputs();
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public boolean checkSystem() {
        System.out.println("Warning: checking LED systems");
        Timer.delay(3);

        writeLedHardware(255, 0, 0);
        Timer.delay(0.4);
        writeLedHardware(0, 255, 0);
        Timer.delay(0.4);
        writeLedHardware(0, 0, 255);
        Timer.delay(0.4);
        writeLedHardware(0, 0, 0);

        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

    public enum RobotStatus {
        ENABLED(0, 255, 0), // green
        DISABLED(255, 103, 0), // orange
        ERROR(255, 0, 0), // red
        AUTONOMOUS(0, 255, 255), // cyan (we can also try 42, 161, 152)
        ENDGAME(0, 0, 255), // blue
        SEEN_TARGET(255, 0, 255), // magenta
        ON_TARGET(255, 0, 20), // deep magenta
        DRIVETRAIN_FLIPPED(255, 255, 0), // yellow
        OFF(0, 0, 0); // off

        int red;
        int green;
        int blue;

        RobotStatus(int r, int g, int b) {
            this.red = r;
            this.green = g;
            this.blue = b;
        }

        public int getRed() {
            return red;
        }

        public int getGreen() {
            return green;
        }

        public int getBlue() {
            return blue;
        }
    }
}
