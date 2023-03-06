// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

import org.firstinspires.ftc.teamcode.common.hardware.Robot

/**
 * This is a advanced opmode for debugging your motor configuration.
 *
 * To use this opmode simply follow the instructions displayed in the telemetry.
 *
 * The opmode will start by turning on one motor for 0.5 seconds, indicate which 
 * motor spun. This will repeat for each motor.
 *
 * At the end change your config based on the output of the opmode.
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
 * Uncomment the @Disabled tag below to use this opmode.
 */
@Disabled
@TeleOp(name = "Motor Assignment Debugger")
class MotorAssignmentDebugger : LinearOpMode() {    
    enum class Motor {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    override fun runOpMode() {
        val robot = Robot(hardwareMap)
        val motorSwapPairs = emptyList<Pair<Motor, Motor>>().toMutableList()

        telemetry.addLine("Ready to start motor assignment debugger")
        telemetry.update()

        waitForStart()

        telemetry.clearAll()
        telemetry.addLine("Starting motor assignment debugger.")
        telemetry.addLine("Running for 0.5 seconds.")
        telemetry.addLine("Press the button corresponding to the motor that spins.")
        telemetry.addLine("Press any button to continue.")
        telemetry.update()

        waitForInput()

        telemetry.clearAll()
        telemetry.addLine("Starting motor assignment debugger.")
        telemetry.addLine("Running front left motor.")
        telemetry.addLine("Press the button corresponding to the motor that spins.")
        telemetry.update()

        robot.drive.setMotorPowers(listOf(0.7, 0.0, 0.0, 0.0))
        sleep(500)
        robot.drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            motorSwapPairs += Pair(Motor.FRONT_LEFT, Motor.FRONT_LEFT)
        } else if (gamepad1.y) {
            motorSwapPairs += Pair(Motor.FRONT_LEFT, Motor.FRONT_RIGHT)
        } else if (gamepad1.b) {
            motorSwapPairs += Pair(Motor.FRONT_LEFT, Motor.BACK_RIGHT)
        } else if (gamepad1.a) {
            motorSwapPairs += Pair(Motor.FRONT_LEFT, Motor.BACK_LEFT)
        }

        telemetry.clearAll()
        telemetry.addLine("Running front right motor.")
        telemetry.addLine("Press the button corresponding to the motor that spins.")
        telemetry.update()

        robot.drive.setMotorPowers(listOf(0.0, 0.7, 0.0, 0.0))
        sleep(500)
        robot.drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            motorSwapPairs += Pair(Motor.FRONT_RIGHT, Motor.FRONT_LEFT)
        } else if (gamepad1.y) {
            motorSwapPairs += Pair(Motor.FRONT_RIGHT, Motor.FRONT_RIGHT)
        } else if (gamepad1.b) {
            motorSwapPairs += Pair(Motor.FRONT_RIGHT, Motor.BACK_RIGHT)
        } else if (gamepad1.a) {
            motorSwapPairs += Pair(Motor.FRONT_RIGHT, Motor.BACK_LEFT)
        }

        telemetry.clearAll()
        telemetry.addLine("Running back right motor.")
        telemetry.addLine("Press the button corresponding to the motor that spins.")
        telemetry.update()

        robot.drive.setMotorPowers(listOf(0.0, 0.0, 0.7, 0.0))
        sleep(500)
        robot.drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            motorSwapPairs += Pair(Motor.BACK_RIGHT, Motor.FRONT_LEFT)
        } else if (gamepad1.y) {
            motorSwapPairs += Pair(Motor.BACK_RIGHT, Motor.FRONT_RIGHT)
        } else if (gamepad1.b) {
            motorSwapPairs += Pair(Motor.BACK_RIGHT, Motor.BACK_RIGHT)
        } else if (gamepad1.a) {
            motorSwapPairs += Pair(Motor.BACK_RIGHT, Motor.BACK_LEFT)
        }

        telemetry.clearAll()
        telemetry.addLine("Running back left motor.")
        telemetry.addLine("Press the button corresponding to the motor that spins.")

        robot.drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.7))
        sleep(500)
        robot.drive.setMotorPowers(listOf(0.0, 0.0, 0.0, 0.0))

        waitForInput()

        if (gamepad1.x) {
            motorSwapPairs += Pair(Motor.BACK_LEFT, Motor.FRONT_LEFT)
        } else if (gamepad1.y) {
            motorSwapPairs += Pair(Motor.BACK_LEFT, Motor.FRONT_RIGHT)
        } else if (gamepad1.b) {
            motorSwapPairs += Pair(Motor.BACK_LEFT, Motor.BACK_RIGHT)
        } else if (gamepad1.a) {
            motorSwapPairs += Pair(Motor.BACK_LEFT, Motor.BACK_LEFT)
        }

        telemetry.clearAll()
        telemetry.addLine("Motor assignment debugger complete.")
        telemetry.addLine("Adjust your configuration accordingly.")

        motorSwapPairs.forEach {
            telemetry.addLine("${it.first} -> ${it.second}")
        }

        telemetry.update()

        while (opModeIsActive()) {
            idle()
        }
    }

    private fun waitForInput() {
        while (!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
            idle()
        }
    }
}
