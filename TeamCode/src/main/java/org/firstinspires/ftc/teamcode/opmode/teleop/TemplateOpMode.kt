// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.opmode.teleop

import com.arcrobotics.ftclib.command.CommandScheduler
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.common.control.OpModeEx
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose
import org.firstinspires.ftc.teamcode.common.hardware.Robot

@Disabled
@TeleOp(name = "Template OpMode")
class TemplateOpMode : OpModeEx() {
    lateinit var robot: Robot

    override fun initialize() {
        CommandScheduler.getInstance().reset()
        
        robot = Robot(hardwareMap)
        robot.startImuThread(this)
    }

    override fun run() {
        robot.drive.setDrivePower(
            Pose(
                -gamepadEx1.leftStickX,
                -gamepadEx1.leftStickY,
                -gamepadEx1.rightStickX
            ).times(0.8 - (0.6 * gamepadEx1.rightTrigger))
        )

        CommandScheduler.getInstance().run()
    }
}
