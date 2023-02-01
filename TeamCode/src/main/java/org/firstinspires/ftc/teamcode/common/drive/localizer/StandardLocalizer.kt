// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT

package org.firstinspires.ftc.teamcode.common.drive.localizer

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.teamcode.common.hardware.Encoder

/**
 * Standard tracking wheel localizer implementation assuming the standard configuration of three tracking wheels:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Adapted from the Roadrunner quickstart.
 */
class StandardLocalizer(hardwareMap: HardwareMap) : ThreeTrackingWheelLocalizer(
        listOf(
                Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0), // left encoder
                Pose2d(0.0, -LATERAL_DISTANCE / 2, 0.0), // right encoder
                Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // center encoder
        )
) {
    companion object {
        private const val TICKS_PER_REV = 8192 // rev through bore encoder
        private const val WHEEL_RADIUS = 1.0 // in
        private const val GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        private const val LATERAL_DISTANCE = 14.0 // in; distance between the left and right wheels
        private const val FORWARD_OFFSET = 0.0 // in; offset of the lateral wheel
    }

    private var leftEncoder: Encoder
    private var rightEncoder: Encoder
    private var centerEncoder: Encoder

    init {
        leftEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "leftEncoder"))
        rightEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "rightEncoder"))
        centerEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "centerEncoder"))
    }

    fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_RADIUS * 2.0 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }

    override fun getWheelPositions(): List<Double> {
        return listOf(
            encoderTicksToInches((leftEncoder.getCurrentPosition()).toDouble()),
            encoderTicksToInches((rightEncoder.getCurrentPosition()).toDouble()),
            encoderTicksToInches((centerEncoder.getCurrentPosition()).toDouble())
        )
    }

    override fun getWheelVelocities(): List<Double> {
        return listOf(
            encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
            encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
            encoderTicksToInches(centerEncoder.getCorrectedVelocity())
        )
    }
}
