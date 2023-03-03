// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.localization

import com.acmerobotics.roadrunner.util.Angle

import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose
import org.firstinspires.ftc.teamcode.common.drive.geometry.Vector

import java.util.function.DoubleSupplier
import java.util.function.IntSupplier

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

/**
 * A localizer that uses two tracking wheels for translation tracking and an
 * outside sensor for rotation detection. 
 */
class TwoWheelLocalizer(
        private val horizontalPosition: IntSupplier,
        horizontalEncoderPlacement: Pose,
        private val verticalPosition: IntSupplier,
        verticalEncoderPlacement: Pose,
        private val heading: DoubleSupplier
) {
    companion object {
        private const val ticksPerEncoderRevolution = 8192
        private const val trackingWheelRadius = 1.0

        var imuOffset = 0.0

        /**
         * Converts traking wheel encoder ticks to inches.
         */
        fun encoderTicksToInches(ticks: Int) =
            trackingWheelRadius * 2 * Math.PI * ticks / ticksPerEncoderRevolution

        /**
         * Thrown when the given positions of the trackingwheels does not allow
         * for localization.
         */
        class InvalidEncoderPlacementException : RuntimeException(
            "The given encoder placement cannot support desired localization."
        )
    }

    private var lastWheelPositions = doubleArrayOf(0.0, 0.0)
    private var lastHeading = Double.NaN

    private val forwardSolver: DecompositionSolver

    private var internalPose = Pose()
    var poseEstimate: Pose
        get() = internalPose
        set(value) {
            internalPose = value
            lastWheelPositions = doubleArrayOf(0.0, 0.0)
            lastHeading = Double.NaN
        }

    init {
        val inverseMatrix = Array2DRowRealMatrix(3, 3)
        val wheelPositions = arrayOf(
            horizontalEncoderPlacement,
            verticalEncoderPlacement
        )

        for (i in 0..1) {
            val orientationVector = wheelPositions[i].headingVector()
            val positionVector = wheelPositions[i].vector()
            inverseMatrix.setEntry(i, 0, orientationVector.x)
            inverseMatrix.setEntry(i, 1, orientationVector.y)
            inverseMatrix.setEntry(i, 2, 
                positionVector.x * orientationVector.y - 
                positionVector.y * orientationVector.x
            )
        }

        inverseMatrix.setEntry(2, 2, 1.0)
        forwardSolver = LUDecomposition(inverseMatrix).solver

        if (forwardSolver.isNonSingular) {
            throw InvalidEncoderPlacementException()
        }
    }

    /**
     * Updates the current pose estimate using the latest encoder readings.
     *
     * This function should be called periodically in a opmode's loop() method.
     */
    fun update() {
        val currentWheelPositions = getWheelPositions()
        val currentHeading = getHeading()

        val wheelDeltas = currentWheelPositions
            .zip(lastWheelPositions)
            .map { it.first - it.second }
        val headingDelta = Angle.normDelta(currentHeading - lastHeading)
        val robotPoseDelta = calculatePoseDelta(wheelDeltas, headingDelta)
        internalPose = relativeOdometryUpdate(internalPose, robotPoseDelta)

        lastWheelPositions = currentWheelPositions
        lastHeading = currentHeading
    }

    /**
     * Resets the current pose estimate to zero.
     */
    fun reset() {
        internalPose = Pose()
        lastWheelPositions = doubleArrayOf(0.0, 0.0)
        lastHeading = Double.NaN
    }

    /**
     * Returns the outside sensor's heading.
     */
    fun getHeading() = heading.asDouble - imuOffset

    /**
     * Returns the current tracking wheel encoder positions.
     */
    private fun getWheelPositions() = doubleArrayOf(
        encoderTicksToInches(horizontalPosition.asInt),
        encoderTicksToInches(verticalPosition.asInt)
    )

    /**
     * Calculates the change in robot pose given the change in wheel positions
     * and the change in heading.
     */
    private fun calculatePoseDelta(
        wheelDeltas: List<Double>, 
        headingDelta: Double
    ): Pose {
        val rawPoseDelta = forwardSolver.solve(
            MatrixUtils.createRealMatrix(
                arrayOf((wheelDeltas + headingDelta).toDoubleArray())
            ).transpose()
        )

        return Pose(
            rawPoseDelta.getEntry(0, 0),
            rawPoseDelta.getEntry(1, 0),
            rawPoseDelta.getEntry(2, 0)
        )
    }

    /**
     * Preforms a relative odometry update given the current robot pose and the
     * change in robot pose.
     *
     * This function assumes that the robot was traveling at a constant
     * velocity and that the change in the overall robot pose is small.
     * 
     * This assumption should always be true if update() is being called
     * periodically.
     */
    private fun relativeOdometryUpdate(
        currentRobotPose: Pose,
        robotPoseDelta: Pose
    ): Pose {
        val (robotSin, robotCos) = if (abs(robotPoseDelta.heading) < 1e-6) {
            1.0 - robotPoseDelta.heading.pow(2) / 6.0 to robotPoseDelta.heading / 2.0
        } else {
            sin(robotPoseDelta.heading) / robotPoseDelta.heading to (1.0 - cos(robotPoseDelta.heading)) / robotPoseDelta.heading
        }

        val newRobotVectorDelta = Vector(
            robotSin * robotPoseDelta.x - robotCos * robotPoseDelta.y,
            robotCos * robotPoseDelta.x + robotSin * robotPoseDelta.y 
        )

        val newRobotPoseDelta = Pose(
            newRobotVectorDelta.rotated(currentRobotPose.heading),
            robotPoseDelta.heading
        )

        return Pose(
            currentRobotPose.x + newRobotPoseDelta.x,
            currentRobotPose.y + newRobotPoseDelta.y,
            Angle.norm(currentRobotPose.heading + newRobotPoseDelta.heading)
        )
    }
}
