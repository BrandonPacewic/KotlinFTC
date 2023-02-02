// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d

import org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment.SequenceSegment

/**
 * Adapted from the Roadrunner quickstart
 */
class TrajectorySequence(private val sequences: MutableList<SequenceSegment>?) {
    init {
        if (sequences.isNullOrEmpty()) {
            throw EmptySequenceException()
        }
    }

    fun start(): Pose2d? {
        return sequences!![0].getStartPose()
    }

    fun end(): Pose2d? {
        return sequences!![sequences.size - 1].getEndPose()
    }

    fun duration(): Double {
        var durationTotal = 0.0

        for (sequence in sequences!!) {
            durationTotal += sequence.getDuration()!!
        }

        return durationTotal
    }

    fun get(sequenceIndex: Int): SequenceSegment {
        return sequences!![sequenceIndex]
    }

    fun size(): Int {
        return sequences!!.size
    }
}
