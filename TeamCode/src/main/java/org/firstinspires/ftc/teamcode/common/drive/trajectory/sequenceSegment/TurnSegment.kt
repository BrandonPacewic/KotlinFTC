// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT

package org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import com.acmerobotics.roadrunner.util.Angle

/**
 * Adapted from the Roadrunner quickstart
 */
class TurnSegment(
    startPose: Pose2d?, 
    private var totalRotation: Double, 
    private var motionProfile: MotionProfile?, 
    markers: MutableList<TrajectoryMarker>?
) : SequenceSegment(
    motionProfile?.duration(),
    startPose,
    Pose2d(
        startPose!!.x, startPose!!.y,
        Angle.norm(startPose.heading + totalRotation)
    ),
    markers
) {

    fun getTotalRotation(): Double {
        return totalRotation
    }

    fun getMotionProfile(): MotionProfile? {
        return motionProfile
    }
}
