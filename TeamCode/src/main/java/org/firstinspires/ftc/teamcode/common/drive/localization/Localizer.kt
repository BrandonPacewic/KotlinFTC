// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.localization

import org.firstinspires.ftc.teamcode.common.geometry.Pose2d

interface Localizer {
    var poseEstimate: Pose2d
    var poseVelocity: Pose2d

    fun update()
}
