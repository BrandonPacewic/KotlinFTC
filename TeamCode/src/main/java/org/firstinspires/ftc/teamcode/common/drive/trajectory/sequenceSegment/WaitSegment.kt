package org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker

/**
 * Adapted from the Roadrunner quickstart
 */
class WaitSegment(
    pose: Pose2d?,
    durationSeconds: Double,
    markers: MutableList<TrajectoryMarker>?
) : SequenceSegment(
    durationSeconds,
    pose,
    pose,
    markers
)
