// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.commandbase.commandbase

import com.arcrobotics.ftclib.command.CommandBase

import org.firstinspires.ftc.teamcode.common.control.PIDController
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose
import org.firstinspires.ftc.teamcode.common.drive.localization.TwoWheelLocalizer

import kotlin.math.abs
import kotlin.math.hypot

/**
 * Drives the robot through the specified points splining them together.
 */
class SplinePositionCommand(
    var drivetrain: Drivetrain,
    var localizer: TwoWheelLocalizer,
    var targetPositions: List<Pose>,    
): CommandBase() {
    private val splineAllowedTranslationalError = 3.0
    private val splineAllowedRotationalError = 5.0

    private val allowedEndingTranslationalError = 0.5
    private val allowedEndingRotationalError = 0.5

    private val maxMotorPower = 0.7

    private val xController = PIDController(0.0, 0.0, 0.0, maxMotorPower)
    private val yController = PIDController(0.0, 0.0, 0.0, maxMotorPower)
    private val headingController = PIDController(0.0, 0.0, 0.0, maxMotorPower)

    private var currentTargetIndex = 0

    override fun execute() {
        val robotPose = localizer.poseEstimate
        val currentTargetPose = targetPositions[currentTargetIndex]

        var controllerOutput = Pose(
            xController.calculate(robotPose.x, currentTargetPose.x),
            yController.calculate(robotPose.y, currentTargetPose.y),
            headingController.calculate(robotPose.heading, currentTargetPose.heading)
        )

        val rotatedVector = controllerOutput.vector().rotated(robotPose.heading)
        controllerOutput = Pose(rotatedVector, controllerOutput.heading)

        controllerOutput.clip(Pose(maxMotorPower, maxMotorPower, maxMotorPower))

        val voltage = drivetrain.voltage

        drivetrain.setDrivePower(Pose(
                (controllerOutput.x / voltage) * 12.5,
                (controllerOutput.y / voltage) * 12.5,
                (controllerOutput.heading / voltage) * 12.5
        ))

        if (currentTargetIndex < targetPositions.size - 1) {
            val error = targetPositions[currentTargetIndex].minus(localizer.poseEstimate)

            val reachedTranslation = hypot(error.x, error.y) < splineAllowedTranslationalError
            val reachedRotation = abs(error.heading) < splineAllowedRotationalError

            if (reachedTranslation && reachedRotation) {
                currentTargetIndex++
            }
        }
    }

    override fun isFinished(): Boolean {
        val error = targetPositions.last().minus(localizer.poseEstimate)

        val reachedTranslation = hypot(error.x, error.y) < allowedEndingTranslationalError
        val reachedRotation = abs(error.heading) < allowedEndingRotationalError

        return reachedTranslation && reachedRotation
    }
}
