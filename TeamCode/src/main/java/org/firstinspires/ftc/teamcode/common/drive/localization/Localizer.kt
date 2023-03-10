// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.localization

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose

interface Localizer {
    /**
     * Holds a static offset value for the heading.
     */
    companion object {
        var headingOffset = 0.0
    }

    /**
     * Returns the current pose estimate.
     */
    val poseEstimate: Pose

    /**
     * Updates the current pose estimate.
     */
    fun update()

    /**
     * Gets the adjusted heading.
     */
    fun getHeading(): Double

    /**
     * Resets the current pose estimate.
     */
    fun reset()
}
