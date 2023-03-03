// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.control

import com.qualcomm.robotcore.util.ElapsedTime

/**
 * Four term time based PIDF controller.
 */
class PIDFController(
    kP: Double,
    kI: Double,
    kD: Double,
    var kF: Double
): PIDController(kP, kI, kD) {
    constructor(
        kP: Double,
        kI: Double,
        kD: Double,
        kF: Double,
        maxOutput: Double,
        minOutput: Double
    ): this(kP, kI, kD, kF) {
        this.maxOutput = maxOutput
        this.minOutput = minOutput
    }

    /**
     * Calculates the output of the PIDF controller based on the current vs
     * target position.
     *
     * IMPORTANT: Once you use this function for a given situation, you must
     * call reset() before changing the set of circumstances that this
     * controller is being used under.
     */
    override fun calculate(currentPosition: Double, targetPosition: Double): Double {
        if (elapsedTime == null) {
            elapsedTime = ElapsedTime()

            return firstCalculate(currentPosition, targetPosition)
        }

        timeStep = elapsedTime!!.milliseconds() - lastTime
        currentError = targetPosition - currentPosition

        val p = kP * currentError
        val i = kI * currentError * timeStep
        val d = (kD * (currentError - lastError)) / timeStep
        val f = kF * targetPosition
        var output = p + i + d + f

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
     * First output calculation for the PIDF controller.
     *
     * Gets called when the first calculate() is called and starts the timer
     * for the controller. The first output is solely based on the proportional
     * and feedforward term.
     */
    override fun firstCalculate(
        currentPosition: Double, 
        targetPosition: Double
    ): Double {
        currentError = targetPosition - currentPosition
        
        val p = kP * currentError
        val f = kF * targetPosition
        var output = p + f

        output = when {
            output > maxOutput -> maxOutput
            output < minOutput -> minOutput
            else -> output
        }

        lastError = currentError
        lastTime = elapsedTime!!.milliseconds()

        return output
    }
}
