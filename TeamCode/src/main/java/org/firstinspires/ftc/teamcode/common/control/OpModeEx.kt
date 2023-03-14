// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.control

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

/**
 * Extends the functionality of LinearOpMode.
 */
abstract class OpModeEx : LinearOpMode() {
    /**
     * Called when the OpMode is initialized.
     */
    abstract fun initialize()

    /**
     * Called in a loop while the OpMode is waiting to be started.
     */
    open fun waitForStartLoop() {
        // Empty by default. Not required.
    }

    /**
     * Called in the first OpMode loop.
     */
    open fun firstRun() {
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
        initialize()

        while (!isStarted && !isStopRequested) {
            waitForStartLoop()
        }

        if (isStopRequested) {
            return
        }

        firstRun()

        while (opModeIsActive() && !isStopRequested) {
            run()
        }

        end()
    }
}
