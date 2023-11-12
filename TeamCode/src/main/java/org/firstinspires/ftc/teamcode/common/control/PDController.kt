// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.control

/**
 * Two term PD controller.
 */
open class PDController(
    kP: Double,
    kD: Double
) : PIDController(kP, 0.0, kD) {
    constructor(
        kP: Double,
        kD: Double,
        maxOutput: Double,
        minOutput: Double
    ) : this(kP, kD) {
        this.maxOutput = maxOutput
        this.minOutput = minOutput
    }

    constructor(
        kP: Double,
        kD: Double,
        maxOutput: Double
    ) : this(kP, kD) {
        this.maxOutput = maxOutput
        this.minOutput = -maxOutput
    }
}
