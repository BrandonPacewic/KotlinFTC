// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.opmode.auto

import com.arcrobotics.ftclib.command.CommandScheduler
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

import org.firstinspires.ftc.teamcode.common.hardware.Robot

@Disabled
@Autonomous(name = "Template Auto")
class TemplateAuto : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(hardwareMap, Robot.OpMode.AUTO)
    }
}
