// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.gamepad

/**
 * Attaches a precise double / boolean state to a gamepadEx trigger.
 */
class TriggerReader(gamepadEx: GamepadEx, trigger: GamepadTrigger, threshold: Double) {
    private var lastState: Boolean
    private var currentState: Boolean
    private var triggerValueSupplier: () -> Double

    constructor(gamepadEx: GamepadEx, trigger: GamepadTrigger) : this(gamepadEx, trigger, 0.5)

    init {
        triggerValueSupplier = { gamepadEx.getTrigger(trigger) }
        currentState = triggerValueSupplier() > threshold
        lastState = currentState
    }

    /**
     * Updates the state of the trigger.
     */
    fun update() {
        lastState = currentState
        currentState = triggerValueSupplier() > threshold
    }

    /**
     * Returns true if the trigger is currently held.
     */
    fun isDown(): Boolean {
        return currentState
    }

    /**
     * Returns true if the trigger was just held.
     */
    fun risingEdge(): Boolean {
        return currentState && !lastState
    }

    /**
     * Returns true if the trigger was just released.
     */
    fun fallingEdge(): Boolean {
        return !currentState && lastState
    }
}
