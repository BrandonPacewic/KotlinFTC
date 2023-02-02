// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.hardware

import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

import kotlin.math.max
import kotlin.math.min
import kotlin.math.round

/**
 * Wraps a motor instance to provide encoder functionality while allowing separate inversion
 * of the motor and encoder.
 *
 * Adapted from the Roadrunner quickstart
 */
class Encoder {
    private var motor: DcMotorEx
    private var clock: NanoClock

    enum class Direction(private var multiplier: Int) {
        FORWARD(1),
        REVERSE(-1);

        fun getMultiplier(): Int {
            return multiplier
        }
    }

    private var direction: Direction

    private var lastPosition: Int
    private var velocityEstimateIndex: Int = 0
    private var velocityEstimates: DoubleArray
    private var lastUpdateTime: Double

    constructor(motor: DcMotorEx, clock: NanoClock) {
        this.motor = motor
        this.clock = clock

        direction = Direction.FORWARD

        lastPosition = 0
        velocityEstimates = DoubleArray(3)
        lastUpdateTime = clock.seconds()
    }

    constructor(motor: DcMotorEx) : this(motor, NanoClock.system())

    fun getDirection(): Direction {
        return direction
    }

    private fun getMultiplier(): Int {
        return getDirection().getMultiplier() * if (motor.direction == DcMotorSimple.Direction.REVERSE) -1 else 1
    }

    /**
     * Sets the direction of the encoder. This does not affect the direction of the connected motor.
     *
     * @param direction the direction of the encoder
     */
     fun setDirection(direction: Direction) {
        this.direction = direction
    }

    /**
     * Gets the position from the underlying motor and adjusts for the set direction.
     * Additionally, updates velocity estimates.
     *
     * @return encoder position
     */
    fun getCurrentPosition(): Int {
        val multiplier = direction.getMultiplier()
        val currentPosition = motor.currentPosition * multiplier

        if (currentPosition == lastPosition) {
            return currentPosition
        }

        val currentTime = clock.seconds()
        val dt = currentTime - lastUpdateTime
        velocityEstimates[velocityEstimateIndex] = (currentPosition - lastPosition) / dt
        velocityEstimateIndex = (velocityEstimateIndex + 1) % velocityEstimates.size
        lastUpdateTime = currentTime
        lastPosition = currentPosition

        return currentPosition
    }

    /**
     * Gets the velocity from the underlying motor and adjusts for the set direction.
     *
    * @return raw encoder velocity
     */
    fun getRawVelocity(): Double {
        return motor.velocity * direction.getMultiplier()
    }

    /**
     * Uses velocity estimates gathered in {@link #getCurrentPosition} to estimate the upper bits of velocity
     * that are lost in overflow due to velocity being transmitted as 16 bits.
     * CAVEAT: must regularly call {@link #getCurrentPosition} for the compensation to work correctly.
     *
     * @return corrected velocity
     */
    fun getCorrectedVelocity(): Double {
        val median = when (velocityEstimates[0] > velocityEstimates[1]) {
            true -> max(velocityEstimates[1], min(velocityEstimates[0], velocityEstimates[2]))
            false -> max(velocityEstimates[0], min(velocityEstimates[1], velocityEstimates[2]))
        }

        return inverseOverflow(getRawVelocity(), median)
    }

    companion object {
        private const val CPS_STEP = 0x10000

        /**
         * Inverse of overflow compensation in {@link DcMotorEx#getVelocity}.
         *
         * For more information read the Roadrunner quickstart.
         *
         * @param rawVelocity raw velocity
         * @param median median of velocity estimates
         * @return corrected velocity
         */
        fun inverseOverflow(rawVelocity: Double, median: Double): Double {
            var real = rawVelocity.toInt() and 0xffff

            real += ((real % 20) / 4) * CPS_STEP
            real += (round((median - real) / (5 * CPS_STEP)) * 5 * CPS_STEP).toInt()

            return real.toDouble()
        }
    }
}
