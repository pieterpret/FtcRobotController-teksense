package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoEx {
    private Servo servo;
    private double openPos;
    private double closePos;
    private boolean isOpen;

    // Constructor-style setup
    public ServoEx(HardwareMap hw, String name, double openPos, double closePos) {
        servo = hw.get(Servo.class, name);
        this.openPos = openPos;
        this.closePos = closePos;
        this.isOpen = false;
        servo.setPosition(closePos); // default closed
    }

    // Basic control
    public void setPosition(double pos) {
        servo.setPosition(pos);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    // Convenience methods
    public void open() {
        servo.setPosition(openPos);
        isOpen = true;
    }

    public void close() {
        servo.setPosition(closePos);
        isOpen = false;
    }

    public void toggle() {
        if (isOpen) {
            close();
        } else {
            open();
        }
    }

}
