// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.control

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

import org.firstinspires.ftc.teamcode.common.gamepad.GamepadEx

/**
 * Extends the functionality of LinearOpMode.
 */
abstract class OpModeEx : LinearOpMode() {
    lateinit var gamepadEx1: GamepadEx
    lateinit var gamepadEx2: GamepadEx

    /**
     * Called when the OpMode is initialized.
     */
    abstract fun initialize()

    /**
     * Called in a loop while the OpMode is waiting to be started.
     */
    open fun initLoop() {
        // Empty by default. Not required.
    }

    /**
     * Called in the first OpMode loop.
     */
    open fun begin() {
        // Empty by default. Not required.
    }

    /**
     * Called each loop of the OpMode.
     */
    abstract fun run()

    /**
     * Called at the end of the OpMode.
     */
    open fun end() {
        // Empty by default. Not required.
    }

    override fun runOpMode() {
        gamepadEx1 = GamepadEx(gamepad1)
        gamepadEx2 = GamepadEx(gamepad2)

        initialize()

        while (!isStarted && !isStopRequested) {
            initLoop()
        }

        if (isStopRequested) {
            return
        }

        begin()

        while (opModeIsActive() && !isStopRequested) {
            gamepadEx1.update()
            gamepadEx2.update()

            run()
        }

        end()
    }
}
