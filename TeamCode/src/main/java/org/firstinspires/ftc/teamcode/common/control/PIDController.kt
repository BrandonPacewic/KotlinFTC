// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.control

/**
 * Three term, time based PID controller.
 */
open class PIDController(
    kP: Double,
    kI: Double,
    kD: Double,
) : PIDFController(kP, kI, kD, 0.0) {
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
    ) : this(kP, kI, kD) {
        this.maxOutput = maxOutput
        this.minOutput = -maxOutput
    }
}
