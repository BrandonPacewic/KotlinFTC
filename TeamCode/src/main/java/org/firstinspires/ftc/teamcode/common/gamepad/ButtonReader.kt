// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.gamepad

/**
 * Attaches boolean state to a gamepadEx button.
 */
class ButtonReader(gamepadEx: GamepadEx, button: GamepadButton) {
    private var lastState: Boolean
    private var currentState: Boolean
    private var buttonStateSupplier: () -> Boolean

    init {
        buttonStateSupplier = { gamepadEx.getButton(button) }
        currentState = buttonStateSupplier()
        lastState = currentState
    }

    /**
     * Updates the state of the button.
     */
    fun update() {
        lastState = currentState
        currentState = buttonStateSupplier()
    }

    /**
     * Returns true if the button is currently pressed.
     */
    fun isDown(): Boolean {
        return currentState
    }

    /**
     * Returns true if the button was just pressed.
     */
    fun risingEdge(): Boolean {
        return currentState && !lastState
    }

    /**
     * Returns true of the button was just released.
     */
    fun fallingEdge(): Boolean {
        return !currentState && lastState
    }
}
