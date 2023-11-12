// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.control

/**
 * One term P controller.
 */
class PController(kP: Double) : PDController(kP, 0.0) {
    constructor(
        kP: Double,
        maxOutput: Double,
        minOutput: Double
    ) : this(kP) {
        this.maxOutput = maxOutput
        this.minOutput = minOutput
    }

    constructor(
        kP: Double,
        maxOutput: Double
    ) : this(kP) {
        this.maxOutput = maxOutput
        this.minOutput = -maxOutput
    }
}
