// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.commandbase.commands

import com.arcrobotics.ftclib.command.CommandBase

import org.firstinspires.ftc.teamcode.common.control.PIDController
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose
import org.firstinspires.ftc.teamcode.common.drive.localization.TwoWheelLocalizer

import kotlin.math.abs
import kotlin.math.hypot

/**
 * Drives the robot to a specified position using PID controllers to determine,
 * the translational and rotational inputs.
 */
class PositionCommand(
    var drivetrain: Drivetrain,
    var localizer: TwoWheelLocalizer,
    var targetPosition: Pose,
): CommandBase() {
    private val allowedTranslationalError = 0.5
    private val allowedRotationalError = 0.5

    private val maxMotorPower = 0.7

    private val xController = PIDController(0.0, 0.0, 0.0, maxMotorPower)
    private val yController = PIDController(0.0, 0.0, 0.0, maxMotorPower)
    private val headingController = PIDController(0.0, 0.0, 0.0, maxMotorPower)

    override fun execute() {
        val robotPose = localizer.poseEstimate

        var controllerOutput = Pose(
            xController.calculate(robotPose.x, targetPosition.x),
            yController.calculate(robotPose.y, targetPosition.y),
            headingController.calculate(robotPose.heading, targetPosition.heading)
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
    }

    override fun isFinished(): Boolean {
        val error = targetPosition.minus(localizer.poseEstimate)

        val reachedTranslation = hypot(error.x, error.y) < allowedTranslationalError
        val reachedHeading = abs(error.heading) < allowedRotationalError

        return reachedTranslation && reachedHeading
    }
}
