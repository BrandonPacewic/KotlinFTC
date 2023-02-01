// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT

package org.firstinspires.ftc.teamcode.common.util

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import kotlin.math.ceil

/**
 * Set of helper function for drawing Trajectories and Paths to the FTC Dashboard
 *
 * Adapted from the Roadrunner quickstart
 */
class DashboardUtil {
    companion object {
        private const val defaultResolution = 2.0F // in
        private const val robotRadius = 9.0F // in
        
        fun drawPoseHistory(canvas: Canvas, poseHistory: List<Pose2d>) {
            val xPoints = DoubleArray(poseHistory.size)
            val yPoints = DoubleArray(poseHistory.size)

            for (i in poseHistory.indices) {
                val pose = poseHistory[i]
                xPoints[i] = pose.x
                yPoints[i] = pose.y
            }

            canvas.strokePolyline(xPoints, yPoints)
        }

        fun drawSampledPath(canvas: Canvas, path: Path, resolution: Double) {
            val samples = ceil(path.length() / resolution).toInt()
            val xPoints = DoubleArray(samples)
            val yPoints = DoubleArray(samples)
            val dx = path.length() / (samples - 1)

            for (i in 0 until samples) {
                val displacement = i * dx.toDouble()
                val pose = path[displacement]
                xPoints[i] = pose.x
                yPoints[i] = pose.y
            }

            canvas.strokePolyline(xPoints, yPoints)
        }
    
        fun drawSampledPath(canvas: Canvas, path: Path) {
            drawSampledPath(canvas, path, defaultResolution.toDouble())
        }
    
        fun drawRobot(canvas: Canvas, pose: Pose2d) {
            canvas.strokeCircle(pose.x, pose.y, robotRadius.toDouble())
            val headingVector: Vector2d = pose.headingVec().times(robotRadius.toDouble())
            val x1 = pose.x + headingVector.x / 2
            val y1 = pose.y + headingVector.y / 2
            val x2 = pose.x + headingVector.x
            val y2 = pose.y + headingVector.y
            canvas.strokeLine(x1, y1, x2, y2)
        }
    }
}
