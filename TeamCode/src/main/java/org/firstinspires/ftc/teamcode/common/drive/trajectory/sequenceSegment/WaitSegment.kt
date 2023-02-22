// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment

import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker

import org.firstinspires.ftc.teamcode.common.geometry.Pose2d

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
