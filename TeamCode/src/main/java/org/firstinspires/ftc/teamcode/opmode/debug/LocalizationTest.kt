// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.opmode.debug

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

import org.firstinspires.ftc.teamcode.common.hardware.Robot

/**
 * Localization test opmode. This opmode will start the robot localizer set
 * in the robot class. It will print out the current state of the localizer
 * components as well as the current robot pose estimate.
 *
 * Remove the @Disabled tag below to use this opmode.
 */
@Disabled
@Autonomous(name = "Localization Test")
class LocalizationTest : LinearOpMode() {
    enum class DigitalState {
        FUNCTIONING,
        PREVIOUSLY_FUNCTIONING,
        NOT_FUNCTIONING
    }

    override fun runOpMode() {
        val robot = Robot(hardwareMap, Robot.OpMode.AUTO)

        telemetry.addLine("Ready to run localization test")
        telemetry.update()

        waitForStart()

        robot.startImuThread(this)

        var imuState = DigitalState.NOT_FUNCTIONING
        var leftEncoderState = DigitalState.NOT_FUNCTIONING
        var rightEncoderState = DigitalState.NOT_FUNCTIONING

        var lastImuReading = robot.getHeading()
        var lastLeftEncoderReading = robot.horizontalEncoder!!.getCurrentPosition()
        var lastRightEncoderReading = robot.lateralEncoder!!.getCurrentPosition()

        while (opModeIsActive()) {
            if (robot.getHeading() == lastImuReading) {
                if (imuState == DigitalState.FUNCTIONING) {
                    imuState = DigitalState.PREVIOUSLY_FUNCTIONING
                } 
            } else {
                imuState = DigitalState.FUNCTIONING
            }

            if (robot.horizontalEncoder!!.getCurrentPosition() == lastLeftEncoderReading) {
                if (leftEncoderState == DigitalState.FUNCTIONING) {
                    leftEncoderState = DigitalState.PREVIOUSLY_FUNCTIONING
                } 
            } else {
                leftEncoderState = DigitalState.FUNCTIONING
            }

            if (robot.lateralEncoder!!.getCurrentPosition() == lastRightEncoderReading) {
                if (rightEncoderState == DigitalState.FUNCTIONING) {
                    rightEncoderState = DigitalState.PREVIOUSLY_FUNCTIONING
                } 
            } else {
                rightEncoderState = DigitalState.FUNCTIONING
            }

            lastImuReading = robot.getHeading()
            lastLeftEncoderReading = robot.horizontalEncoder!!.getCurrentPosition()
            lastRightEncoderReading = robot.lateralEncoder!!.getCurrentPosition()

            telemetry.addData("IMU: ", robot.getHeading())
            telemetry.addData("Horisontal Encoder: ", robot.horizontalEncoder!!.getCurrentPosition())
            telemetry.addData("Lateral Encoder: ", robot.lateralEncoder!!.getCurrentPosition())
            telemetry.addData("X: ", robot.localizer!!.poseEstimate.x)
            telemetry.addData("Y: ", robot.localizer!!.poseEstimate.y)
            telemetry.addData("Heading: ", robot.localizer!!.poseEstimate.heading)
            telemetry.addData("IMU State: ", imuState)
            telemetry.addData("Left Encoder State: ", leftEncoderState)
            telemetry.addData("Right Encoder State: ", rightEncoderState)
            telemetry.update()
        }
    }
}
