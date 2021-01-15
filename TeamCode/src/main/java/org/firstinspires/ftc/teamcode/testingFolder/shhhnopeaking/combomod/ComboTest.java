package org.firstinspires.ftc.teamcode.testingFolder.shhhnopeaking.combomod;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.function.BooleanSupplier;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class ComboTest {

    public static int x = 3;
    private Gamepad gamepad;
    private ComboGamepad comboGamepad;
    private GamepadEx gamepadEx;

    @BeforeEach
    public void setup(){
        gamepad = new Gamepad();
        comboGamepad = new ComboGamepad(gamepad);
        gamepadEx = new GamepadEx(gamepad);
    }

    @Test
    public void simpleTest() {
        BooleanSupplier wasJustPressed = () -> comboGamepad.wasJustPressed(GamepadKeys.Button.A);
        gamepad.a = false;
        assertFalse(comboGamepad.getButton(GamepadKeys.Button.A));
        gamepad.a = true;
        assertTrue(comboGamepad.getButton(GamepadKeys.Button.A));
        comboGamepad.readButtons();
        assertTrue(wasJustPressed.getAsBoolean());
    }

    @Test
    public void whenPressedTest() {
        gamepad.a = false;
        BooleanSupplier wasJustPressed = () -> comboGamepad.wasJustPressed(GamepadKeys.Button.A);
        assertFalse(wasJustPressed.getAsBoolean());
        gamepad.a = true;
        assertFalse(wasJustPressed.getAsBoolean());
        comboGamepad.readButtons();
        assertTrue(wasJustPressed.getAsBoolean());
        gamepad.a = true;
        assertTrue(wasJustPressed.getAsBoolean());
        comboGamepad.readButtons();
        assertFalse(wasJustPressed.getAsBoolean());
        gamepad.a = false;
        assertFalse(wasJustPressed.getAsBoolean());
        comboGamepad.readButtons();
        assertFalse(wasJustPressed.getAsBoolean());
    }
}
