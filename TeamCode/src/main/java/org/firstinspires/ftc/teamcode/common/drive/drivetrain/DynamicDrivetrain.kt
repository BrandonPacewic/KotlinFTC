// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.drivetrain

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

import kotlin.collections.HashMap

abstract class DynamicDrivetrain(hardwareMap: HardwareMap) : Drivetrain(hardwareMap) {
    companion object {
        var driveMotors: HashMap<String, String> = hashMapOf(
            "frontLeft" to "dm1",
            "frontRight" to "dm2",
            "backRight" to "dm4",
            "backLeft" to "dm3"
        )

        var driveMotorDirections: HashMap<String, DcMotorSimple.Direction> = hashMapOf(
            "frontLeft" to DcMotorSimple.Direction.REVERSE,
            "frontRight" to DcMotorSimple.Direction.FORWARD,
            "backRight" to DcMotorSimple.Direction.FORWARD,
            "backLeft" to DcMotorSimple.Direction.REVERSE
        )
    }
}
