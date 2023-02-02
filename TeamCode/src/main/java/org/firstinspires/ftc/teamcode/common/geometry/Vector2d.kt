// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.geometry

import org.firstinspires.ftc.teamcode.common.util.AngleUtil
import org.firstinspires.ftc.teamcode.common.util.MathUtil

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class Vector2d(
    val x: Double = 0.0,
    val y: Double = 0.0
) {
    companion object {
        fun polar(magnitude: Double, angle: Double): Vector2d {
            return Vector2d(
                magnitude * cos(angle),
                magnitude * sin(angle)
            )
        }
    }

    fun normalize(): Double {
        return sqrt(x * x + y * y)
    }

    fun angle(): Double {
        return AngleUtil.normalize(atan2(y, x))
    }

    operator fun plus(other: Vector2d): Vector2d {
        return Vector2d(
            x + other.x,
            y + other.y
        )
    }

    operator fun minus(other: Vector2d): Vector2d {
        return Vector2d(
            x - other.x,
            y - other.y
        )
    }

    operator fun times(scalar: Double): Vector2d {
        return Vector2d(
            x * scalar,
            y * scalar
        )
    }

    operator fun div(scalar: Double): Vector2d {
        return Vector2d(
            x / scalar,
            y / scalar
        )
    }

    operator fun unaryMinus(): Vector2d {
        return Vector2d(-x, -y)
    }

    fun dot(other: Vector2d): Double {
        return x * other.x + y * other.y
    }

    fun distanceTo(other: Vector2d): Double {
        return (other - this).normalize()
    }

    fun projectOnto(other: Vector2d): Vector2d {
        return other * (this.dot(other) / other.dot(other))
    }

    fun rotate(angle: Double): Vector2d {
        return Vector2d(
            x * cos(angle) - y * sin(angle),
            x * sin(angle) + y * cos(angle)
        )
    }

    fun epsilonEquals(other: Vector2d): Boolean {
        return MathUtil.epsilonEquals(x, other.x) &&
                MathUtil.epsilonEquals(y, other.y)
    }

    override fun toString(): String {
        return String.format("(%.3f, %.3f)", x, y)
    }
}
