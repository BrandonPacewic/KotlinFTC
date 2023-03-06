// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.opmode.teleop

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose
import org.firstinspires.ftc.teamcode.common.hardware.Robot

@Disabled
@TeleOp(name = "Template OpMode")
class TemplateOpMode : CommandOpMode() {
    lateinit var robot: Robot

    override fun initialize() {
        CommandScheduler.getInstance().reset()
        
        robot = Robot(hardwareMap)
        robot.startImuThread(this)
    }

    override fun run() {
        robot.drive.setDrivePower(
            Pose(
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            ).times(0.8 - (0.6 * gamepad1.right_trigger.toDouble()))
        )

        CommandScheduler.getInstance().run()
    }

    override fun reset() {
        CommandScheduler.getInstance().reset()
        robot.reset()
    }
}
