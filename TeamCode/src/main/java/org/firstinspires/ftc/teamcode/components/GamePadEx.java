package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
 *                  GamepadEx Helper Class
 *
 * Purpose:
 *   Provides an abstraction layer for FTC gamepad inputs.
 *   Copies joystick, trigger, and button states into simple variables/booleans
 *   so OpMode logic can be written cleanly without direct gamepad calls.
 *
 * Benefits:
 *   - Keeps FTC SDK gamepad references separate from control logic
 *   - Makes code easier to read, maintain, and refactor
 *   - Allows extension (e.g. edge detection, toggles, macros)
 *
 * Usage:
 *   GamepadEx driver = new GamepadEx(gamepad1);
 *   driver.update(gamepad1);
 *   if (driver.a) { ... }   // instead of gamepad1.a
 *
 * Notes:
 *   Call update() once per loop to refresh values.
 */
public class GamePadEx {
    // Joysticks
    public double leftStickX;
    public double leftStickY;
    public double rightStickX;
    public double rightStickY;

    // Triggers
    public double leftTrigger;
    public double rightTrigger;

    // Buttons
    public boolean a;
    public boolean b;
    public boolean x;
    public boolean y;
    public boolean dpadUp;
    public boolean dpadDown;
    public boolean dpadLeft;
    public boolean dpadRight;
    public boolean leftBumper;
    public boolean rightBumper;

    // Constructor
    public GamePadEx(Gamepad gp) {
        update(gp);
    }

    // Update values each loop
    public void update(Gamepad gp) {
        leftStickX  = gp.left_stick_x;
        leftStickY  = gp.left_stick_y;
        rightStickX = gp.right_stick_x;
        rightStickY = gp.right_stick_y;

        leftTrigger  = gp.left_trigger;
        rightTrigger = gp.right_trigger;

        a = gp.a;
        b = gp.b;
        x = gp.x;
        y = gp.y;

        dpadUp    = gp.dpad_up;
        dpadDown  = gp.dpad_down;
        dpadLeft  = gp.dpad_left;
        dpadRight = gp.dpad_right;

        leftBumper  = gp.left_bumper;
        rightBumper = gp.right_bumper;
    }

}
