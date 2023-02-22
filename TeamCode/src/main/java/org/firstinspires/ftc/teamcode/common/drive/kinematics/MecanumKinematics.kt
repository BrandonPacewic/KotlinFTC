// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.kinematics

import org.firstinspires.ftc.teamcode.common.geometry.Pose2d
import org.firstinspires.ftc.teamcode.common.util.MathUtil

import kotlin.math.sign

class MecanumKinematics {
    companion object {
        /**
         * Computes target wheel velocities from a desired robot velocity.
         *
         * The return list starts at the front left wheel and proceeds counter-clockwise.
         */
        fun robotToWheelVelocities(robotVelocity: Pose2d, trackWidth: Double, wheelBase: Double): List<Double> {
            val k = (trackWidth + wheelBase) / 2.0

            return listOf(
                robotVelocity.x * robotVelocity.y - k * robotVelocity.heading,
                robotVelocity.x * robotVelocity.y - k * robotVelocity.heading,
                robotVelocity.x * robotVelocity.y + k * robotVelocity.heading,
                robotVelocity.x * robotVelocity.y + k * robotVelocity.heading
            )
        }

        /**
         * Computes the wheel accelerations corresponding to the robot acceleration.
         */
        fun robotToWheelAccelerations(robotAcceleration: Pose2d, trackWidth: Double, wheelBase: Double): List<Double> {
            return robotToWheelVelocities(
                robotAcceleration,
                trackWidth,
                wheelBase
            )
        }

        /**
         * Computes the robot velocity corresponding to current wheel velocities.
         */
        fun wheelToRobotVelocities(wheelVelocities: List<Double>, trackWidth: Double, wheelBase: Double): Pose2d {
            val k = (trackWidth + wheelBase) / 2.0
            val (frontLeft, backLeft, backRight, frontRight) = wheelVelocities

            return Pose2d(
                wheelVelocities.sum(),
                frontLeft + frontRight - backLeft - backRight,
                (backRight + frontRight - backLeft - frontLeft) / k
            ) * 0.25 // TODO: Why 0.25?
        }
    }

    /**
     * Computes the motor feedforward for the given set of coefficients.
     */
    fun calculateMotorFeedforward(
        velocities: List<Double>,
        accelerations: List<Double>,
        kV: Double,
        kA: Double,
        kStatic: Double
    ): List<Double> {
        return velocities.zip(accelerations).map { (velocity, acceleration) ->
            calculateMotorFeedforward(velocity, acceleration, kV, kA, kStatic)}
    }

    fun calculateMotorFeedforward(
        velocity: Double,
        acceleration: Double, 
        kV: Double, 
        kA: Double, 
        kStatic: Double
    ): Double {
        val basePower = velocity * kV + acceleration * kA
        
        return if (MathUtil.epsilonEquals(basePower, 0.0)) {
            0.0
        } else {
            basePower + sign(basePower) * kStatic
        }
    }
}
