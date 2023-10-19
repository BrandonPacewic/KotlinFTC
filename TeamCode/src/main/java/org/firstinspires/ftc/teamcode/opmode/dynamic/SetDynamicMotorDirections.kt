// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.opmode.dynamic

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple

import org.firstinspires.ftc.teamcode.common.drive.drivetrain.DynamicDrivetrain
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.DynamicMecanumDrivetrain

/**
 * This is a advanced opmode for configuring the dynamic drivetrain system.
 *
 * The opmode will start by runing on one motor for 0.5 seconds, indicate which
 * direction that particular motor should spin for the robot to have moved forward.
 * This will repeat for each motor.
 *
 * At the end the dynamic drivetrain will have it's motor direction assignments updated.
 *
 * Button Mappings:
 *
 * Xbox/PS4 Button - Direction
 *  X / ▢         - Forward
 *  A / X         - Reverse
 *
 * Remove the @Disabled tag below to use this opmode.
 */
@Disabled
@TeleOp(name = "Establish dynamic motor directions")
class SetDynamicMotorDirections : LinearOpMode() {
    override fun runOpMode() {
        val drive: DynamicDrivetrain

        telemetry.addLine("Ready to begin dynamic motor direction assignment")
        telemetry.update()

        waitForStart()

        telemetry.clearAll()
        telemetry.addLine("What drivetrain would you like to configure?")
        telemetry.addLine("x: Mecanum")
        telemetry.update()

        waitForInput()
        telemetry.clearAll()

        if (gamepad1.x) {
            drive = DynamicMecanumDrivetrain(hardwareMap)
            telemetry.addLine("Mecanum drivetrain selected")
        } else {
            telemetry.addLine("Input failed")
            telemetry.update()
            return
        }

        telemetry.addLine("Starting dynamic motor direction assignment")
        telemetry.addLine("Running for 0.5 seconds")
        telemetry.addLine("Press the button corresponding to the direction that the motor should spin")
        telemetry.addLine("Press any button to continue.")
        telemetry.update()

        waitForInput()

        telemetry.clearAll()
        telemetry.addLine("Running...")
        telemetry.update()

        drive.setMotorPowers(listOf(0.7, 0.0, 0.0, 0.0))
        sleep(500)
        drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            DynamicDrivetrain.driveMotorDirections["frontLeft"] = DcMotorSimple.Direction.FORWARD
        } else if (gamepad1.y) {
            DynamicDrivetrain.driveMotorDirections["frontLeft"] = DcMotorSimple.Direction.REVERSE
        } else {
            telemetry.addLine("Input failed")
            telemetry.update()
            return
        }

        drive.setMotorPowers(listOf(0.0, 0.7, 0.0, 0.0))
        sleep(500)
        drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            DynamicDrivetrain.driveMotorDirections["frontRight"] = DcMotorSimple.Direction.FORWARD
        } else if (gamepad1.y) {
            DynamicDrivetrain.driveMotorDirections["frontRight"] = DcMotorSimple.Direction.REVERSE
        } else {
            telemetry.addLine("Input failed")
            telemetry.update()
            return
        }

        drive.setMotorPowers(listOf(0.0, 0.0, 0.7, 0.0))
        sleep(500)
        drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            DynamicDrivetrain.driveMotorDirections["backRight"] = DcMotorSimple.Direction.FORWARD
        } else if (gamepad1.y) {
            DynamicDrivetrain.driveMotorDirections["backRight"] = DcMotorSimple.Direction.REVERSE
        } else {
            telemetry.addLine("Input failed")
            telemetry.update()
            return
        }

        drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.7))
        sleep(500)
        drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            DynamicDrivetrain.driveMotorDirections["backLeft"] = DcMotorSimple.Direction.FORWARD
        } else if (gamepad1.y) {
            DynamicDrivetrain.driveMotorDirections["backLeft"] = DcMotorSimple.Direction.REVERSE
        } else {
            telemetry.addLine("Input failed")
            telemetry.update()
            return
        }

        telemetry.clearAll()
        telemetry.addLine("Dynamic motor direction assignment complete")
        telemetry.update()
    }

    private fun waitForInput() {
        while (!gamepad1.x && !gamepad1.y && !gamepad1.b && !gamepad1.a) {
            idle()
        }
    }
}
