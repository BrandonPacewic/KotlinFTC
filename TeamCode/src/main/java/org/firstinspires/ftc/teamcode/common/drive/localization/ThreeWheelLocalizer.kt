// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.localization

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition

import org.firstinspires.ftc.teamcode.common.drive.kinematics.LocalizationKinematics
import org.firstinspires.ftc.teamcode.common.geometry.Pose2d
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
 * @param wheelPoses list of tracking wheel poses realtive to the robot's center.
 * @param hardwareMap Hardware map of the current opMode.
 */
class ThreeWheelLocalizer(wheelPoses: List<Pose2d>, hardwareMap: HardwareMap) : Localizer {
    companion object {
        private const val ticksPerRevolution = 8192.0
        private const val wheelRadius = 1.0 // in
        private const val gearRatio = 1.0
    }

    private var leftEncoder: Encoder
    private var rightEncoder: Encoder
    private var centerEncoder: Encoder

    private var internalPoseEstimate: Pose2d = Pose2d()
    override var poseEstimate: Pose2d
        get() = internalPoseEstimate
        set(value) {
            internalPoseEstimate = value
            lastWheelPositions = emptyList()
        }
    override var poseVelocity: Pose2d = Pose2d()

    private var lastWheelPositions = emptyList<Double>()

    private val forwardSolver: DecompositionSolver

    init {
        leftEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "leftEncoder"))
        rightEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "rightEncoder"))
        centerEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "centerEncoder"))

        if (wheelPoses.size != 3) {
            throw InvalidLocalizerWheelException("Localizer wheel data must have 3 wheels")
        }

        val inverseMatrix = Array2DRowRealMatrix(3, 3)

        for (i in 0..2) {
            val orientationVector = wheelPoses[i].headingVector
            val positionVector = wheelPoses[i].vector

            inverseMatrix.setEntry(i, 0, orientationVector.x)
            inverseMatrix.setEntry(i, 1, orientationVector.y)
            inverseMatrix.setEntry(
                i, 2, 
                positionVector.x * orientationVector.y - positionVector.y * orientationVector.x
            )
        }

        forwardSolver = LUDecomposition(inverseMatrix).solver

        if (!forwardSolver.isNonSingular) {
            throw InvalidLocalizerWheelException("The specified tracking wheel configuration cannot support full localization")
        }
    }

    fun calculatePoseDelta(wheelDeltas: List<Double>): Pose2d {
        val rawPoseDelta = forwardSolver.solve(
            Array2DRowRealMatrix(
                doubleArrayOf(
                    wheelDeltas[0],
                    wheelDeltas[1],
                    wheelDeltas[2]
                )
            ).transpose()
        )

        return Pose2d(
            rawPoseDelta.getEntry(0, 0),
            rawPoseDelta.getEntry(1, 0),
            rawPoseDelta.getEntry(2, 0)
        )
    }

    override fun update() {
        val wheelPositions = getWheelPositions()

        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                .zip(lastWheelPositions)
                .map { it.first - it.second }
            
            val poseDelta = calculatePoseDelta(wheelDeltas)
            internalPoseEstimate = LocalizationKinematics.relativeOdometryUpdate(internalPoseEstimate, poseDelta)
        }

        poseVelocity = calculatePoseDelta(getWheelVelocities())

        lastWheelPositions = wheelPositions
    }

    /**
     * Returns the positions of the three tracking wheels in inches (not encoder ticks).
     */
    fun getWheelPositions(): List<Double> {
        return listOf(
            encoderTicksToInches(leftEncoder.getCurrentPosition().toDouble()),
            encoderTicksToInches(rightEncoder.getCurrentPosition().toDouble()),
            encoderTicksToInches(centerEncoder.getCurrentPosition().toDouble())
        )
    }

    /**
     * Returns the velocities of the three tracking wheels in inches per second (not encoder ticks per second).
     */
    fun getWheelVelocities(): List<Double> {
        return listOf(
            encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
            encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
            encoderTicksToInches(centerEncoder.getCorrectedVelocity())
        )
    }

    private fun encoderTicksToInches(ticks: Double): Double {
        return wheelRadius * 2.0 * Math.PI * gearRatio * ticks / ticksPerRevolution
    }
}
