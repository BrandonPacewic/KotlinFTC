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
                -gamepadEx1.leftStickX,
                -gamepadEx1.leftStickY,
                -gamepadEx1.rightStickX
            ).times(0.8 - (0.6 * gamepadEx1.rightTrigger))
        )
    }
}
