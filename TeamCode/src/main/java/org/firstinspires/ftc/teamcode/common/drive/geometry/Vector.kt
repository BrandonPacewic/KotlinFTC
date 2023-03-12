// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.geometry

import kotlin.math.cos
import kotlin.math.sin

/**
 * Represents a 2D vector; simply a point in 2D space.
 */
data class Vector(var x: Double = 0.0, var y: Double = 0.0) {
    fun plus(other: Vector) = Vector(x + other.x, y + other.y)
    fun minus(other: Vector) = Vector(x - other.x, y - other.y)
    fun times(scalar: Double) = Vector(x * scalar, y * scalar)
    fun div(scalar: Double) = Vector(x / scalar, y / scalar)

    /**
     * Rotates this vector by a given angle.
     */
    fun rotated(angle: Double) = Vector(
        x * cos(angle) - y * sin(angle),
        x * sin(angle) + y * cos(angle)
    )

    override fun toString() = String.format("(%.3f, %.3f)", x, y)
}
