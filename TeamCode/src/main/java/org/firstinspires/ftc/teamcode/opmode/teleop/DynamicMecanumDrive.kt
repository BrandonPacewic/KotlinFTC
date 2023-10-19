// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.common.control.OpModeEx
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.DynamicMecanumDrivetrain
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose

@Disabled
@TeleOp(name = "Dynamic Mecanum Drive")
class DynamicMecanumDrive : OpModeEx() {
    lateinit var drive: DynamicMecanumDrivetrain

    override fun initialize() {
        drive = DynamicMecanumDrivetrain(hardwareMap)
    }

    override fun run() {
        drive.setDrivePower(
            Pose(
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            ).times(0.8 - (0.6 * gamepad1.right_trigger.toDouble()))
        )
    }
}
