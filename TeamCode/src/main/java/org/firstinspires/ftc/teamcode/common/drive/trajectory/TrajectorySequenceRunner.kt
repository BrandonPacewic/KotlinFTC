// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT

package org.firstinspires.ftc.teamcode.common.drive.trajectory

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import com.acmerobotics.roadrunner.util.NanoClock

import org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment.SequenceSegment
import org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment.TrajectorySegment
import org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment.TurnSegment
import org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment.WaitSegment
import org.firstinspires.ftc.teamcode.common.util.DashboardUtil

import java.util.LinkedList

/**
 * Adapted from the Roadrunner quickstart
 */
@Config
class TrajectorySequenceRunner {
    val COLOR_INACTIVE_TRAJECTORY = "#4caf507a";
    val COLOR_INACTIVE_TURN = "#7c4dff7a";
    val COLOR_INACTIVE_WAIT = "#dd2c007a";

    val COLOR_ACTIVE_TRAJECTORY = "#4CAF50";
    val COLOR_ACTIVE_TURN = "#7c4dff";
    val COLOR_ACTIVE_WAIT = "#dd2c00";

    val COLOR_TARGET_POSE = "#4CAF50"
    val COLOR_POSE_HISTORY = "#3F51B5"
    val COLOR_POSE_ESTIMATE = "#3F51B5"

    val POSE_HISTORY_LIMIT = 100

    var follower: TrajectoryFollower?

    var turnController: PIDFController?

    var clock: NanoClock

    var currentTrajectorySequence: TrajectorySequence? = null
    var currentSegmentStartTime: Double = 0.0
    var currentSegmentIndex: Int = 0
    var lastSegmentIndex: Int = 0

    var lastPoseError = Pose2d()

    var remainingMarkers = mutableListOf<TrajectoryMarker>()

    private var dashboard: FtcDashboard?
    private var poseHistory = LinkedList<Pose2d>()

    constructor(follower: TrajectoryFollower, headingPIDCoefficients: PIDCoefficients) {
        this.follower = follower

        this.turnController = PIDFController(headingPIDCoefficients)
        this.turnController!!.setInputBounds(0.0, 2 * Math.PI)

        this.clock = NanoClock.system()

        this.dashboard = FtcDashboard.getInstance()
        this.dashboard!!.telemetryTransmissionInterval = 25
    }
    
    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence) {
        currentTrajectorySequence = trajectorySequence
        currentSegmentStartTime = clock.seconds()
        currentSegmentIndex = 0
        lastSegmentIndex = -1
    }

    fun update(poseEstimate: Pose2d, poseVelocity: Pose2d): DriveSignal? {
        var targetPose: Pose2d? = null
        var driveSignal: DriveSignal? = null

        val telemetryPacket = TelemetryPacket()
        val fieldOverlay = telemetryPacket.fieldOverlay()

        var currentSegment: SequenceSegment? = null

        if (currentTrajectorySequence != null) {
            if (currentSegmentIndex >= currentTrajectorySequence!!.size()) {
                for (marker in remainingMarkers) {
                    marker.callback.onMarkerReached()
                }

                remainingMarkers.clear()
                currentTrajectorySequence = null
            }

            if (currentTrajectorySequence == null) {
                return DriveSignal()
            }

            // Current time is stored in a variable to prevent drift.
            val now = clock.seconds()
            val isNewTransition = currentSegmentIndex != lastSegmentIndex

            currentSegment = currentTrajectorySequence!!.get(currentSegmentIndex)

            if (isNewTransition) {
                currentSegmentStartTime = now
                lastSegmentIndex = currentSegmentIndex

                for (marker in remainingMarkers) {
                    marker.callback.onMarkerReached()
                }

                remainingMarkers.clear()

                currentSegment.getMarkers()?.let { remainingMarkers.addAll(it) }
                
                // Sort the remaining markers by time.
                remainingMarkers.sortBy { it.time }
            }

            val deltaTime = now - currentSegmentStartTime

            when (currentSegment) {
                is TrajectorySegment -> {
                    val currentTrajectory = (currentSegment.getTrajectory())

                    if (isNewTransition) {
                        follower!!.followTrajectory(currentTrajectory as Trajectory)
                    }

                    if (!follower!!.isFollowing()) {
                        currentSegmentIndex++

                        driveSignal = DriveSignal()
                    } else {
                        driveSignal = follower!!.update(poseEstimate, poseVelocity)
                        lastPoseError = follower!!.lastError
                    }

                    targetPose = currentTrajectory?.get(deltaTime)
                }
                is TurnSegment -> {
                    val targetState: MotionState? = currentSegment.getMotionProfile()?.get(deltaTime)

                    turnController!!.targetPosition = targetState!!.x

                    val correction = turnController!!.update(poseEstimate.heading)
                    val targetOmega = targetState.v
                    val targetAlpha = targetState.a

                    lastPoseError = Pose2d(0.0, 0.0, turnController!!.lastError)

                    val startPose = currentSegment.getStartPose()
                    targetPose = startPose!!.copy(startPose.x, startPose.y, targetState.x)

                    driveSignal = DriveSignal(
                        Pose2d(0.0, 0.0, targetOmega + correction),
                        Pose2d(0.0, 0.0, targetAlpha)
                    )

                    if (deltaTime >= currentSegment.getDuration()!!) {
                        currentSegmentIndex++
                        driveSignal = DriveSignal()
                    }
                }
                is WaitSegment -> {
                    lastPoseError = Pose2d()
                    targetPose = currentSegment.getStartPose()
                    driveSignal = DriveSignal()

                    if (deltaTime >= currentSegment.getDuration()!!) {
                        currentSegmentIndex++
                    }
                }
            }

            // Remove any markers that have been passed.
            while (remainingMarkers.isNotEmpty() && remainingMarkers[0].time <= deltaTime) {
                remainingMarkers[0].callback.onMarkerReached()
                remainingMarkers.removeAt(0)
            }
        }
        
        poseHistory.add(poseEstimate)

        if (poseHistory.size > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst()
        }

        telemetryPacket.put("x", poseEstimate.x)
        telemetryPacket.put("y", poseEstimate.y)
        telemetryPacket.put("heading (deg)", Math.toDegrees(poseEstimate.heading))

        telemetryPacket.put("xError", lastPoseError.x)
        telemetryPacket.put("yError", lastPoseError.y)
        telemetryPacket.put("headingError (deg)", Math.toDegrees(lastPoseError.heading))

        draw(fieldOverlay, currentTrajectorySequence, currentSegment, targetPose, poseEstimate)

        dashboard!!.sendTelemetryPacket(telemetryPacket)

        return driveSignal
    }

    private fun draw(
        fieldOverlay: Canvas,
        trajectorySequence: TrajectorySequence?,
        currentSegment: SequenceSegment?,
        targetPose: Pose2d?,
        poseEstimate: Pose2d
    ) {
        if (trajectorySequence == null || currentSegment == null || targetPose == null) {
            return
        }

        for (i in 0 until trajectorySequence.size()) {
            when (val segment = trajectorySequence.get(i)) {
                is TrajectorySegment -> {
                    fieldOverlay.setStrokeWidth(1)
                    fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY)

                    DashboardUtil.drawSampledPath(fieldOverlay, segment.getTrajectory()!!.path)
                }
                is TurnSegment -> {
                    val startPose = segment.getStartPose()

                    fieldOverlay.setFill(COLOR_INACTIVE_TURN)
                    fieldOverlay.fillCircle(startPose!!.x, startPose!!.y, 2.0)
                }
                is WaitSegment -> {
                    val startPose = segment.getStartPose()

                    fieldOverlay.setStrokeWidth(1)
                    fieldOverlay.setStroke(COLOR_INACTIVE_WAIT)
                    fieldOverlay.strokeCircle(startPose!!.x, startPose!!.y, 3.0)
                }
            }
        }

        when (currentSegment) {
            is TrajectorySegment -> {
                val currentTrajectory = currentSegment.getTrajectory()

                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke(COLOR_ACTIVE_TRAJECTORY)

                DashboardUtil.drawSampledPath(fieldOverlay, currentTrajectory!!.path)
            }
            is TurnSegment -> {
                val startPose = currentSegment.getStartPose()

                fieldOverlay.setFill(COLOR_ACTIVE_TURN)
                fieldOverlay.fillCircle(startPose!!.x, startPose!!.y, 3.0)
            }
            is WaitSegment -> {
                val startPose = currentSegment.getStartPose()

                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke(COLOR_ACTIVE_WAIT)
                fieldOverlay.strokeCircle(startPose!!.x, startPose!!.y, 3.0)
            }
        }

        fieldOverlay.setStrokeWidth(1)
        fieldOverlay.setStroke(COLOR_TARGET_POSE)
        DashboardUtil.drawRobot(fieldOverlay, targetPose)

        fieldOverlay.setStroke(COLOR_POSE_HISTORY)
        DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory)

        fieldOverlay.setStroke(COLOR_POSE_ESTIMATE)
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate)
    }

    fun isBusy(): Boolean {
        return currentTrajectorySequence != null
    }
}
