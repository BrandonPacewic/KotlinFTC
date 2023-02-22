// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.geometry

/**
 * Represents a 2D pose; contains a position and heading.
 */
data class Pose(
    val x: Double = 0.0, 
    val y: Double = 0.0, 
    val heading: Double = 0.0
) {
    constructor(vector: Vector, heading: Double = 0.0) 
        : this(vector.x, vector.y, heading)

    fun vector() = Vector(x, y)

    fun plus(other: Pose) = 
        Pose(x + other.x, y + other.y, heading + other.heading)
    fun minus(other: Pose) = 
        Pose(x - other.x, y - other.y, heading - other.heading)
    fun times(scalar: Double) = Pose(x * scalar, y * scalar, heading * scalar)
    fun div(scalar: Double) = Pose(x / scalar, y / scalar, heading / scalar)

    override fun toString() = String.format("(%.3f, %.3f, %.3f)", x, y, heading)
}
