// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.control

import com.qualcomm.robotcore.util.ElapsedTime

/**
 * Three term time based PID controller.
 */
open class PIDController(
    var kP: Double,
    var kI: Double,
    var kD: Double
) {
    var currentError = 0.0
    var lastError = 0.0

    var timeStep = 0.0
    var lastTime = 0.0
    var elapsedTime: ElapsedTime? = null

    var maxOutput = 1.0
    var minOutput = -1.0

    constructor(
        kP: Double, 
        kI: Double, 
        kD: Double, 
        maxOutput: Double, 
        minOutput: Double
    ) : this(kP, kI, kD) {
        this.maxOutput = maxOutput
        this.minOutput = minOutput
    }

    constructor(
        kP: Double, 
        kI: Double, 
        kD: Double, 
        maxOutput: Double
    ) : this(kP, kI, kD, maxOutput, -maxOutput)

    /**
     * Calculates the output of the PID controller based on the current vs
     * target position.
     *
     * IMPORTANT: Once you use this function for a given situation, you must
     * call reset() before changing the set of circumstances that this
     * controller is being used under.
     */
    open fun calculate(
        currentPosition: Double, 
        targetPosition: Double
    ): Double {
        if (elapsedTime == null) {
            elapsedTime = ElapsedTime()
            
            return firstCalculate(currentPosition, targetPosition)
        }

        timeStep = elapsedTime!!.milliseconds() - lastTime
        currentError = targetPosition - currentPosition

        val p = kP * currentError
        val i = kI * currentError * timeStep
        val d = (kD * (currentError - lastError)) / timeStep
        var output = p + i + d

        output = when {
            output > maxOutput -> maxOutput
            output < minOutput -> minOutput
            else -> output
        }

        lastError = currentError
        lastTime = elapsedTime!!.milliseconds()

        return output
    }

    /**
     * First output calculation for the PID controller.
     *
     * Gets called when the first calculate() is called and starts the timer
     * for the controller. The first output is solely based on the proportional
     * term.
     */
    open fun firstCalculate(
        currentPosition: Double, 
        targetPosition: Double
    ): Double {
        // First time calculate is called, we don't have a time step or last error
        // so we just return the proportional term.
        val currentError = targetPosition - currentPosition
        var output = kP * currentError

        output = when {
            output > maxOutput -> maxOutput
            output < minOutput -> minOutput
            else -> output
        }

        lastError = currentError
        lastTime = elapsedTime!!.milliseconds()

        return output
    }

    /**
     * Resets the PID controller.
     *
     * Only call this function when you want to completely reset the controller.
     * This function should only be called when the task that the controller was
     * initialized for is complete and being changed.
     */
    fun reset() {
        currentError = 0.0
        lastError = 0.0
        timeStep = 0.0
        lastTime = 0.0
        elapsedTime = null
    }
}
