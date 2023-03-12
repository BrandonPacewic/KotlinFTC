// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.geometry

import kotlin.math.cos
import kotlin.math.sin

/**
 * Represents a 2D pose; contains a position and heading.
 */
data class Pose(
    var x: Double = 0.0,
    var y: Double = 0.0,
    var heading: Double = 0.0
) {
    constructor(vector: Vector, heading: Double = 0.0) 
        : this(vector.x, vector.y, heading)

    /**
     * Returns the vector representation of this pose.
     */
    fun vector() = Vector(x, y)

    /**
     * Returns the heading vector of this pose.
     */
    fun headingVector() = Vector(cos(heading), sin(heading))

    fun plus(other: Pose) = 
        Pose(x + other.x, y + other.y, heading + other.heading)
    fun minus(other: Pose) = 
        Pose(x - other.x, y - other.y, heading - other.heading)
    fun times(scalar: Double) = Pose(x * scalar, y * scalar, heading * scalar)
    fun div(scalar: Double) = Pose(x / scalar, y / scalar, heading / scalar)

    /**
     * Clips this pose to the given min and max values.
     */
    fun clip(min: Pose, max: Pose) {
        if (x < min.x) x = min.x
        if (y < min.y) y = min.y
        if (heading < min.heading) heading = min.heading

        if (x > max.x) x = max.x
        if (y > max.y) y = max.y
        if (heading > max.heading) heading = max.heading
    }

    fun clip(clip: Pose) {
        clip(Pose(-clip.x, -clip.y, -clip.heading), clip)
    }

    override fun toString() = String.format("(%.3f, %.3f, %.3f)", x, y, heading)
}
