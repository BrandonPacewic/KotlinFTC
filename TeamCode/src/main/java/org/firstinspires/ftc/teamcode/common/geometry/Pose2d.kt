// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.geometry

import org.firstinspires.ftc.teamcode.common.util.AngleUtil
import org.firstinspires.ftc.teamcode.common.util.MathUtil

import kotlin.math.cos
import kotlin.math.sin

data class Pose2d(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val heading: Double = 0.0
) {
    constructor(pose: Vector2d, heading: Double) : this(pose.x, pose.y, heading)

    val vector: Vector2d
        get() = Vector2d(x, y)

    val headingVector: Vector2d
        get() = Vector2d(cos(heading), sin(heading))

    operator fun plus(other: Pose2d): Pose2d {
        return Pose2d(
            x + other.x,
            y + other.y,
            heading + other.heading
        )
    }

    operator fun minus(other: Pose2d): Pose2d {
        return Pose2d(
            x - other.x,
            y - other.y,
            heading - other.heading
        )
    }

    operator fun times(scalar: Double): Pose2d {
        return Pose2d(
            x * scalar,
            y * scalar,
            heading * scalar
        )
    }

    operator fun div(scalar: Double): Pose2d {
        return Pose2d(
            x / scalar,
            y / scalar,
            heading / scalar
        )
    }

    fun epsilonEquals(other: Pose2d): Boolean {
        return MathUtil.epsilonEquals(x, other.x) &&
                MathUtil.epsilonEquals(y, other.y) &&
                MathUtil.epsilonEquals(AngleUtil.normalizeDelta(heading - other.heading), 0.0)
    }

    override fun toString(): String {
        return String.format("(%.3f, %.3f, %.3f)", x, y, Math.toDegrees(heading))
    }
}
