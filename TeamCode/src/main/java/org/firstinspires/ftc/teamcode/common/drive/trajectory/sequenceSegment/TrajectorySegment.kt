// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT

package org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment

import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import com.acmerobotics.roadrunner.trajectory.Trajectory

/**
 * Adapted from the Roadrunner quickstart
 */
class TrajectorySegment(
    private var trajectory: Trajectory?
) : SequenceSegment(
    trajectory?.duration(),
    trajectory?.start(),
    trajectory?.end(),
    mutableListOf<TrajectoryMarker>()
) {

    fun getTrajectory(): Trajectory? {
        return this.trajectory
    }
}
