// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.control

import com.qualcomm.robotcore.util.ElapsedTime

/**
 * Four term, time based PIDF controller.
 */
open class PIDFController(
    var kP: Double,
    var kI: Double,
    var kD: Double,
    var kF: Double
) {
    var currentError = 0.0
    var lastError = 0.0

    var lastTimeStamp = 0.0
    var elapsedTime = ElapsedTime()

    var maxOutput = 0.0
    var minOutput = 0.0

    constructor(
        kP: Double,
        kI: Double,
        kD: Double,
        kF: Double,
        maxOutput: Double,
        minOutput: Double
    ) : this(kP, kI, kD, kF) {
        this.maxOutput = maxOutput
        this.minOutput = minOutput
    }

    constructor(
        kP: Double,
        kI: Double,
        kD: Double,
        kF: Double,
        maxOutput: Double
    ) : this (kP, kI, kD, kF, maxOutput, -maxOutput)

    /**
     * Calculates the output of the PID controller based on the current vs
     * target position.
     */
    fun calculate(currentPosition: Double, targetPosition: Double): Double {
        lastError = currentError

        val currentTimeStamp = elapsedTime.milliseconds()

        if (lastTimeStamp == 0.0) {
            lastTimeStamp = currentTimeStamp
        }

        val timeStep = currentTimeStamp - lastTimeStamp
        val p = kP * currentError
        val i = kI * currentError * timeStep
        val d = kD * ((currentError - lastError) / timeStep)
        val f = kF * targetPosition
        val output = p + i + d + f

        lastError = currentError
        lastTimeStamp = currentTimeStamp

        return if (maxOutput != 0.0 && minOutput != 0.0) {
            output.coerceIn(minOutput, maxOutput)
        } else {
            output
        }
    }
}
