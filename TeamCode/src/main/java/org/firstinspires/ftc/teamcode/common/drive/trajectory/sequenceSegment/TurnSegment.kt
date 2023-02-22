// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment

import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import com.acmerobotics.roadrunner.util.Angle

import org.firstinspires.ftc.teamcode.common.geometry.Pose2d

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
