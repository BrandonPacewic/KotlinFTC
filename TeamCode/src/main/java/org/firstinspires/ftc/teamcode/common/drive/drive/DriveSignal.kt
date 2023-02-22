// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.drive

import org.firstinspires.ftc.teamcode.common.geometry.Pose2d

/**
 * Signal indicating the commanded kinematic state of a drive.
 *
 * Taken directly from the Roadrunner path following library.
 */
data class DriveSignal(
    val velocity: Pose2d = Pose2d(),
    val acceleration: Pose2d = Pose2d(),
)
