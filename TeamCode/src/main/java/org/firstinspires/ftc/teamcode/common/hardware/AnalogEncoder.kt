// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.hardware

import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.robotcore.hardware.AnalogInput

import kotlin.math.abs

class AnalogEncoder(private val encoder: AnalogInput) {
    enum class Direction {
        FORWARD,
        REVERSE
    }

    private var lastPosition = 0.0

    var direction = Direction.FORWARD
    var offset = 0.0

    var analogRange = 3.3

    constructor(encoder: AnalogInput, analogRange: Double) : this(encoder) {
        this.analogRange = analogRange
    }

    /**
     * Returns an AnalogEncoder with the zero offset set.
     *
     * This simplifies the setup process of the swerve drivetrain modules.
     */
    fun zero(offset: Double): AnalogEncoder {
        this.offset = offset
        return this
    }

    /**
     * Returns the current position of the encoder.
     */
    fun getCurrentPosition(): Double {
        var position = if (direction == Direction.FORWARD) {
            1.0 - encoder.voltage / analogRange
        } else {
            encoder.voltage / analogRange
        }

        position *= (Math.PI * 2.0) - offset

        // Checks for incorrect values when close to zero.
        if (abs(Angle.normDelta(lastPosition)) > 0.1 || abs(Angle.normDelta(position)) < 1.0) {
            lastPosition = position
        }

        return lastPosition
    }
}
