// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.hardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.teamcode.common.drive.drive.mecanum.MecanumDrivetrain
import org.firstinspires.ftc.teamcode.common.drive.localization.TwoWheelLocalizer

class Robot(hardwareMap: HardwareMap, private val opMode: OpMode = OpMode.TELEOP) {
    enum class OpMode {
        TELEOP,
        AUTO
    }

    var drive: MecanumDrivetrain
    var localizer: TwoWheelLocalizer? = null

    init {
        drive = MecanumDrivetrain(hardwareMap)

        if (opMode == OpMode.AUTO) {
            // localizer = TwoWheelLocalizer()
        }
    }
}
