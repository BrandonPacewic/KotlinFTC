// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.gamepad

import com.qualcomm.robotcore.hardware.Gamepad

import kotlin.collections.HashMap

/**
 * An extension of the FTC Gamepad class for more advanced functionality.
 */
class GamepadEx(private var gamepad: Gamepad) {
    val a: Boolean
        get() = gamepad.a
    val b: Boolean
        get() = gamepad.b
    val x: Boolean
        get() = gamepad.x
    val y: Boolean
        get() = gamepad.y
    val dpadUp: Boolean
        get() = gamepad.dpad_up
    val dpadDown: Boolean
        get() = gamepad.dpad_down
    val dpadLeft: Boolean
        get() = gamepad.dpad_left
    val dpadRight: Boolean
        get() = gamepad.dpad_right
    val leftBumper: Boolean
        get() = gamepad.left_bumper
    val rightBumper: Boolean
        get() = gamepad.right_bumper
    val leftStickButton: Boolean
        get() = gamepad.left_stick_button
    val rightStickButton: Boolean
        get() = gamepad.right_stick_button
    val back: Boolean
        get() = gamepad.back
    val start: Boolean
        get() = gamepad.start

    val leftStickX: Double
        get() = gamepad.left_stick_x.toDouble()
    val leftStickY: Double
        get() = gamepad.left_stick_y.toDouble()
    val rightStickX: Double
        get() = gamepad.right_stick_x.toDouble()
    val rightStickY: Double
        get() = gamepad.right_stick_y.toDouble()

    val leftTrigger: Double
        get() = gamepad.left_trigger.toDouble()
    val rightTrigger: Double
        get() = gamepad.right_trigger.toDouble()

    private var buttonReaders: HashMap<GamepadButton, ButtonReader> = hashMapOf()
    private var triggerReaders: HashMap<GamepadTrigger, TriggerReader> = hashMapOf()

    init {
        enumValues<GamepadButton>().forEach {
            buttonReaders[it] = ButtonReader(this, it)
        }

        enumValues<GamepadTrigger>().forEach {
            triggerReaders[it] = TriggerReader(this, it)
        }
    }

    /**
     * Updates the value for each button and trigger reader.
     *
     * This should be called once in each loop of the OpMode.
     */
    fun update() {
        buttonReaders.values.forEach { it.update() }
        triggerReaders.values.forEach { it.update() }
    }

    /**
     * Get the value(s) of the wrapped gamepad's button(s).
     */
    fun getButton(button: GamepadButton): Boolean {
        return when (button) {
            GamepadButton.Y -> gamepad.y
            GamepadButton.X -> gamepad.x
            GamepadButton.A -> gamepad.a
            GamepadButton.B -> gamepad.b
            GamepadButton.LEFT_BUMPER -> gamepad.left_bumper
            GamepadButton.RIGHT_BUMPER -> gamepad.right_bumper
            GamepadButton.BACK -> gamepad.back
            GamepadButton.START -> gamepad.start
            GamepadButton.DPAD_UP -> gamepad.dpad_up
            GamepadButton.DPAD_DOWN -> gamepad.dpad_down
            GamepadButton.DPAD_LEFT -> gamepad.dpad_left
            GamepadButton.DPAD_RIGHT -> gamepad.dpad_right
            GamepadButton.LEFT_STICK_BUTTON -> gamepad.left_stick_button
            GamepadButton.RIGHT_STICK_BUTTON -> gamepad.right_stick_button
        }
    }

    /**
     * Get the value(s) of the wrapped gamepad's trigger(s).
     */
    fun getTrigger(trigger: GamepadTrigger): Double {
        return when (trigger) {
            GamepadTrigger.LEFT_TRIGGER -> gamepad.left_trigger.toDouble()
            GamepadTrigger.RIGHT_TRIGGER -> gamepad.right_trigger.toDouble()
        }
    }

    /**
     * Returns true if the button is currently pressed.
     */
    fun isDown(button: GamepadButton): Boolean {
        return buttonReaders.getValue(button).isDown()
    }

    /**
     * Returns true if the trigger is currently held.
     */
    fun isDown(trigger: GamepadTrigger): Boolean {
        return triggerReaders.getValue(trigger).isDown()
    }

    /**
     * Returns true if the button was just pressed.
     */
    fun risingEdge(button: GamepadButton): Boolean {
        return buttonReaders.getValue(button).risingEdge()
    }

    /**
     * Returns true if the trigger was just held.
     */
    fun risingEdge(trigger: GamepadTrigger): Boolean {
        return triggerReaders.getValue(trigger).risingEdge()
    }

    /**
     * Returns true of the button was just released.
     */
    fun fallingEdge(button: GamepadButton): Boolean {
        return buttonReaders.getValue(button).fallingEdge()
    }

    /**
     * Returns true if the trigger was just released.
     */
    fun fallingEdge(trigger: GamepadTrigger): Boolean {
        return triggerReaders.getValue(trigger).fallingEdge()
    }
}
