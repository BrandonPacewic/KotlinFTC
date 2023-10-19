// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.opmode.dynamic

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

import org.firstinspires.ftc.teamcode.common.drive.drivetrain.DynamicDrivetrain
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.DynamicMecanumDrivetrain

/**
 * This is a advanced opmode for configuring the dynamic drivetrain system.
 *
 * The opmode will start by turning on one motor for 0.5 seconds, indicate which 
 * motor spun. This will repeat for each motor.
 *
 * At the end the dynamic drivetrain will have it's motor assignments updated.
 *
 * Button Mappings:
 *
 * Xbox/PS4 Button - Motor
 *   X / ▢         - Front Left
 *   Y / Δ         - Front Right
 *   B / O         - Rear  Right
 *   A / X         - Rear  Left
 *
 * The buttons are mapped to match the wheels spatially if you
 * were to rotate the gamepad 45deg°. x/square is the front left
 * and each button corresponds to the wheel as you go clockwise.
 *
 *                   / ______ \
 *     ------------.-'   _  '-..+              Front of Bot
 *              /   _  ( Y )  _  \                  ^
 *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
 *        ___  '.      ( A )     /|       Wheel       \      Wheel
 *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 *     |       |                 |                      \
 *      '.___.' '.               |          Rear Left    \   Rear Right
 *               '.             /             Wheel       \    Wheel
 *                \.          .'              (A/X)        \   (B/O)
 *                  \________/
 *
 * Remove the @Disabled tag below to use this opmode.
 */
@Disabled
@TeleOp(name = "Establish dynamic motor assignments")
class SetDynamicMotorAssignments : LinearOpMode() {
    override fun runOpMode() {
        val drive: DynamicDrivetrain

        telemetry.addLine("Ready to begin dynamic motor assignment")
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

        telemetry.addLine("Starting dynamic motor assignment")
        telemetry.addLine("Running for 0.5 seconds")
        telemetry.addLine("Press the button corresponding to the motor that spins")
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

        if (gamepad1.y) {
            DynamicDrivetrain.driveMotors["frontLeft"] = "dm2"
        } else if (gamepad1.b) {
            DynamicDrivetrain.driveMotors["frontLeft"] = "dm3"
        } else if (gamepad1.a) {
            DynamicDrivetrain.driveMotors["frontLeft"] = "dm4"
        }

        drive.setMotorPowers(listOf(0.0, 0.7, 0.0, 0.0))
        sleep(500)
        drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            DynamicDrivetrain.driveMotors["frontRight"] = "dm1"
        } else if (gamepad1.b) {
            DynamicDrivetrain.driveMotors["frontRight"] = "dm3"
        } else if (gamepad1.a) {
            DynamicDrivetrain.driveMotors["frontRight"] = "dm4"
        }

        drive.setMotorPowers(listOf(0.0, 0.0, 0.7, 0.0))
        sleep(500)
        drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            DynamicDrivetrain.driveMotors["backRight"] = "dm1"
        } else if (gamepad1.y) {
            DynamicDrivetrain.driveMotors["backRight"] = "dm2"
        } else if (gamepad1.a) {
            DynamicDrivetrain.driveMotors["backRight"] = "dm4"
        }

        drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.7))
        sleep(500)
        drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            DynamicDrivetrain.driveMotors["backLeft"] = "dm1"
        } else if (gamepad1.y) {
            DynamicDrivetrain.driveMotors["backLeft"] = "dm2"
        } else if (gamepad1.b) {
            DynamicDrivetrain.driveMotors["backLeft"] = "dm3"
        }

        telemetry.clearAll()
        telemetry.addLine("Dynamic motor assignment complete")
        telemetry.update()
    }

    private fun waitForInput() {
        while (!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
            idle()
        }
    }
}
