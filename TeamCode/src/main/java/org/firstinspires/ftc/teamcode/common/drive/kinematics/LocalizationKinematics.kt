// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.kinematics

import org.firstinspires.ftc.teamcode.common.geometry.Pose2d
import org.firstinspires.ftc.teamcode.common.util.AngleUtil
import org.firstinspires.ftc.teamcode.common.util.MathUtil

import kotlin.math.cos
import kotlin.math.sin

class LocalizationKinematics {
    companion object {
        fun relativeOdometryUpdate(currentPose: Pose2d, poseDelta: Pose2d): Pose2d {
            val deltaTheta = poseDelta.heading
            val (sinTerm, cosTerm) = if (MathUtil.epsilonEquals(deltaTheta, 0.0)) {
                1.0 - deltaTheta * deltaTheta / 6.0 to deltaTheta / 2.0
            } else {
                sin(deltaTheta) / deltaTheta to (1 - cos(deltaTheta)) / deltaTheta
            }

            val fieldPositionDelta = Vector2d(
                poseDelta.x * sineTerm - poseDelta.y * cosTerm,
                poseDelta.x * cosTerm + poseDelta.y * sineTerm
            )

            val fieldPoseDelta = Pose2d(
                fieldPositionDelta.rotated(currentPose.heading), poseDelta.heading
            )

            return Pose2d(
                currentPose.x + fieldPoseDelta.x,
                currentPose.y + fieldPoseDelta.y,
                AngleUtil.normalize(currentPose.heading + fieldPoseDelta.heading)
            )
        }
    }
}
