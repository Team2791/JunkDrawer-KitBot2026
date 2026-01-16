package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

    private static final int LED_PORT = 0; // PWM port
    private static final int LED_COUNT = 60; // number of LEDs on strip

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public enum Mode {
        DISABLED,
        TELEOP,
        AUTONOMOUS,
        INTAKING,
        HAS_PIECE,
        SHOOTING,
        ERROR,
        DRIVING,
        TARGETlOCKED,
        NOTDRIVING,
        CALIBRATING
    }

    private Mode currentMode = Mode.DISABLED;

    public LEDs() {
        led = new AddressableLED(LED_PORT);
        buffer = new AddressableLEDBuffer(LED_COUNT);
        led.setLength(LED_COUNT);
        led.start();
    }

    public void setMode(Mode mode) {
        currentMode = mode;
    }

    @Override
    public void periodic() {
        switch (currentMode) {
            case DISABLED -> solid(0, 0, 255); // Blue
            case TELEOP -> solid(0, 255, 0); // Green
            case AUTONOMOUS -> solid(255, 165, 0); // Orange
            case INTAKING -> blink(255, 255, 0); // Yellow
            case HAS_PIECE -> solid(0, 255, 255); // Cyan
            case SHOOTING -> blink(255, 0, 0); // Red
            case ERROR -> rainbow();
            case DRIVING -> solid(10, 67, 10); // different shade of green
            case NOTDRIVING -> solid(67, 10, 10);// different shade of red
            case TARGETlOCKED -> solid(67, 0, 67);// purple
            case CALIBRATING -> solid(67, 79, 90);// light bluhh
        }
        led.setData(buffer);
    }

    private void solid(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    private void blink(int r, int g, int b) {
        boolean on = (System.currentTimeMillis() / 300) % 2 == 0;
        if (on)
            solid(r, g, b);
        else
            solid(0, 0, 0);
    }

    private void rainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (i * 180 / buffer.getLength()) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }
    }
}