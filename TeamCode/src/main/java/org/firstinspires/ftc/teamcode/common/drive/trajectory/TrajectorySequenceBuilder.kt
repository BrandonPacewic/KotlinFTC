// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.trajectory

import com.acmerobotics.roadrunner.path.PathContinuityViolationException
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.DisplacementMarker
import com.acmerobotics.roadrunner.trajectory.DisplacementProducer
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.SpatialMarker
import com.acmerobotics.roadrunner.trajectory.TemporalMarker
import com.acmerobotics.roadrunner.trajectory.TimeProducer
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.util.Angle

import org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment.SequenceSegment
import org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment.TrajectorySegment
import org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment.TurnSegment
import org.firstinspires.ftc.teamcode.common.drive.trajectory.sequenceSegment.WaitSegment
import org.firstinspires.ftc.teamcode.common.geometry.Pose2d
import org.firstinspires.ftc.teamcode.common.geometry.Vector2d

import kotlin.math.abs
import kotlin.math.min

/**
 * Adapted from the Roadrunner quickstart
 */
class TrajectorySequenceBuilder {
    private var resolution = 0.25

    private var baseVelocityConstraint: TrajectoryVelocityConstraint? = null
    private var baseAccelerationConstraint: TrajectoryAccelerationConstraint? = null

    private var currentVelocityConstraint: TrajectoryVelocityConstraint? = null
    private var currentAccelerationConstraint: TrajectoryAccelerationConstraint? = null

    private var baseTurnConstraintMaxAngularVelocity: Double? = null
    private var baseTurnConstraintMaxAngularAcceleration: Double? = null

    private var currentTurnConstraintMaxAngularVelocity: Double? = null
    private var currentTurnConstraintMaxAngularAcceleration: Double? = null

    private var sequenceSegments = mutableListOf<SequenceSegment>()

    private var temporalMarkers = mutableListOf<TemporalMarker>()
    private var displacementMarkers = mutableListOf<DisplacementMarker>()
    private var spatialMarkers = mutableListOf<SpatialMarker>()

    private var lastPose: Pose2d? = null

    private var tangentOffset: Double? = null

    private var setAbsoluteTangent: Boolean? = null
    private var absoluteTangent: Double? = null

    private var currentTrajectoryBuilder: TrajectoryBuilder? = null

    private var currentDuration: Double? = null
    private var currentDisplacement: Double? = null

    private var lastTrajectoryDuration: Double? = null
    private var lastTrajectoryDisplacement: Double? = null

    constructor(
        startPose: Pose2d,
        startTangent: Double?,
        baseVelocityConstraint: TrajectoryVelocityConstraint?,
        baseAccelerationConstraint: TrajectoryAccelerationConstraint?,
        baseTurnConstraintMaxAngularVelocity: Double,
        baseTurnConstraintMaxAngularAcceleration: Double
    ) {
        this.baseVelocityConstraint = baseVelocityConstraint
        this.baseAccelerationConstraint = baseAccelerationConstraint

        this.currentVelocityConstraint = baseVelocityConstraint
        this.currentAccelerationConstraint = baseAccelerationConstraint

        this.baseTurnConstraintMaxAngularVelocity = baseTurnConstraintMaxAngularVelocity
        this.baseTurnConstraintMaxAngularAcceleration = baseTurnConstraintMaxAngularAcceleration

        this.currentTurnConstraintMaxAngularVelocity = baseTurnConstraintMaxAngularVelocity
        this.currentTurnConstraintMaxAngularAcceleration = baseTurnConstraintMaxAngularAcceleration

        this.lastPose = startPose

        this.tangentOffset = 0.0

        if (startTangent == null) {
            this.setAbsoluteTangent = false
            absoluteTangent = 0.0
        } else {
            this.setAbsoluteTangent = true
            absoluteTangent = startTangent
        }

        currentDuration = 0.0
        currentDisplacement = 0.0

        lastTrajectoryDuration = 0.0
        lastTrajectoryDisplacement = 0.0
    }

    constructor(
        startPose: Pose2d,
        baseVelocityConstraint: TrajectoryVelocityConstraint?,
        baseAccelerationConstraint: TrajectoryAccelerationConstraint?,
        baseTurnConstraintMaxAngularVelocity: Double,
        baseTurnConstraintMaxAngularAcceleration: Double
    ) : this(
        startPose,
        null,
        baseVelocityConstraint,
        baseAccelerationConstraint,
        baseTurnConstraintMaxAngularVelocity,
        baseTurnConstraintMaxAngularAcceleration
    )

    fun lineTo(endPosition: Vector2d): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineTo(
                    endPosition, 
                    currentVelocityConstraint, 
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun lineTo(
        endPosition: Vector2d,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineTo(
                        endPosition,
                        velocityConstraint,
                        accelerationConstraint
                )
            }
        })
    }

    fun lineToConstantHeading(endPosition: Vector2d): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToConstantHeading(
                    endPosition,
                    currentVelocityConstraint,
                    currentAccelerationConstraint    
                )
            }
        })
    }

    fun lineToConstantHeading(
        endPosition: Vector2d,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToConstantHeading(
                    endPosition,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun lineToLinearHeading(endPose: Pose2d): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToLinearHeading(
                    endPose,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun lineToLinearHeading(
        endPose: Pose2d,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToLinearHeading(
                    endPose,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun lineToSplineHeading(endPose: Pose2d): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToSplineHeading(
                    endPose,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun lineToSplineHeading(
        endPose: Pose2d,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToSplineHeading(
                    endPose,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun strafeTo(endPosition: Vector2d): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeTo(
                    endPosition,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun strafeTo(
        endPosition: Vector2d,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeTo(
                    endPosition,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun forward(distance: Double): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.forward(
                    distance,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun forward(
        distance: Double,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.forward(
                    distance,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun back(distance: Double): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.back(
                    distance,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun back(
        distance: Double,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.back(
                    distance,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun strafeLeft(distance: Double): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeLeft(
                    distance,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun strafeLeft(
        distance: Double,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeLeft(
                    distance,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun strafeRight(distance: Double): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeRight(
                    distance,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun strafeRight(
        distance: Double,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeRight(
                    distance,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun splineTo(endPosition: Vector2d, endHeading: Double): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineTo(
                    endPosition,
                    endHeading,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun splineTo(
        endPosition: Vector2d,
        endHeading: Double,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineTo(
                    endPosition,
                    endHeading,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun splineToConstantHeading(endPosition: Vector2d, endHeading: Double): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToConstantHeading(
                    endPosition,
                    endHeading,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun splineToConstantHeading(
        endPosition: Vector2d,
        endHeading: Double,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToConstantHeading(
                    endPosition,
                    endHeading,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun splineToLinearHeading(endPosition: Pose2d, endHeading: Double): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToLinearHeading(
                    endPosition,
                    endHeading,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun splineToLinearHeading(
        endPosition: Pose2d,
        endHeading: Double,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToLinearHeading(
                    endPosition,
                    endHeading,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    fun splineToSplineHeading(endPosition: Pose2d, endHeading: Double): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToSplineHeading(
                    endPosition,
                    endHeading,
                    currentVelocityConstraint,
                    currentAccelerationConstraint
                )
            }
        })
    }

    fun splineToSplineHeading(
        endPosition: Pose2d,
        endHeading: Double,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToSplineHeading(
                    endPosition,
                    endHeading,
                    velocityConstraint,
                    accelerationConstraint
                )
            }
        })
    }

    private fun addPath(callback: AddPathCallback): TrajectorySequenceBuilder {
        if (currentTrajectoryBuilder == null) {
            newPath()
        }

        try {
            callback.run()
        // Path continuity is almost expected to be violated, this is the primary behavior of trajectory sequences.
        // When path continuity is violated, we simply create a new path and continue.
        } catch (e: PathContinuityViolationException) {
            newPath()
            callback.run()
        }

        val builtTrajectory = currentTrajectoryBuilder?.build()

        val durationDifference = builtTrajectory!!.duration() - lastTrajectoryDuration!!
        val displacementDifference = builtTrajectory.path.length() - lastTrajectoryDisplacement!!

        lastPose = builtTrajectory.end()
        currentDuration = currentDuration?.plus(durationDifference)
        currentDisplacement = currentDisplacement?.plus(displacementDifference)

        lastTrajectoryDuration = builtTrajectory.duration()
        lastTrajectoryDisplacement = builtTrajectory.path.length()

        return this
    }

    fun setTangent(tangent: Double): TrajectorySequenceBuilder {
        this.setAbsoluteTangent = true
        this.absoluteTangent = tangent

        this.pushPath()

        return this
    }

    fun setTangentOffset(tangentOffset: Double): TrajectorySequenceBuilder {
        this.setAbsoluteTangent = true
        this.absoluteTangent = tangentOffset

        this.pushPath()

        return this
    }

    fun setReversed(reversed: Boolean): TrajectorySequenceBuilder {
        return this.setTangentOffset(if (reversed) Math.PI else 0.0)
    }

    fun setConstraints(
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        this.currentVelocityConstraint = velocityConstraint
        this.currentAccelerationConstraint = accelerationConstraint

        return this
    }

    fun setVelocityConstraint(velocityConstraint: TrajectoryVelocityConstraint): TrajectorySequenceBuilder {
        this.currentVelocityConstraint = velocityConstraint

        return this
    }

    fun resetVelocityConstraint(): TrajectorySequenceBuilder {
        this.currentVelocityConstraint = this.baseVelocityConstraint

        return this
    }

    fun setAccelerationConstraint(accelerationConstraint: TrajectoryAccelerationConstraint): TrajectorySequenceBuilder {
        this.currentAccelerationConstraint = accelerationConstraint

        return this
    }

    fun resetAccelerationConstraint(): TrajectorySequenceBuilder {
        this.currentAccelerationConstraint = this.baseAccelerationConstraint

        return this
    }

    fun setTurnConstraint(maxAngularVelocity: Double, maxAngularAcceleration: Double): TrajectorySequenceBuilder {
        this.currentTurnConstraintMaxAngularVelocity = maxAngularVelocity
        this.currentTurnConstraintMaxAngularAcceleration = maxAngularAcceleration

        return this
    }

    fun resetTurnConstraint(): TrajectorySequenceBuilder {
        this.currentTurnConstraintMaxAngularVelocity = this.baseTurnConstraintMaxAngularVelocity
        this.currentTurnConstraintMaxAngularAcceleration = this.baseTurnConstraintMaxAngularAcceleration

        return this
    }

    fun addTemporalMarker(callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addTemporalMarker(currentDuration!!, callback)
    }

    fun addTemporalMarker(time: Double, callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addTemporalMarker(0.0, time, callback)
    }

    fun addTemporalMarker(scale: Double, offset: Double, callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addTemporalMarker({time -> scale * time + offset}, callback)
    }

    fun addTemporalMarker(timeProducer: TimeProducer, callback: MarkerCallback): TrajectorySequenceBuilder {
        this.temporalMarkers.add(TemporalMarker(timeProducer, callback))

        return this
    }

    fun addSpatialMarker(point: Vector2d, callback: MarkerCallback): TrajectorySequenceBuilder {
        this.spatialMarkers.add(SpatialMarker(point, callback))

        return this
    }

    fun addDisplacementMarker(callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addDisplacementMarker(currentDisplacement!!, callback)
    }

    fun addDisplacementMarker(displacement: Double, callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addDisplacementMarker(0.0, displacement, callback)
    }

    fun addDisplacementMarker(scale: Double, offset: Double, callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addDisplacementMarker({displacement -> scale * displacement + offset}, callback)
    }

    fun addDisplacementMarker(displacementProducer: DisplacementProducer, callback: MarkerCallback): TrajectorySequenceBuilder {
        this.displacementMarkers.add(DisplacementMarker(displacementProducer, callback))

        return this
    }

    fun turn(angle: Double): TrajectorySequenceBuilder {
        return this.turn(angle, currentTurnConstraintMaxAngularVelocity!!, currentTurnConstraintMaxAngularAcceleration!!)
    }

    fun turn(angle: Double, maxAngularVelocity: Double, maxAngularAcceleration: Double): TrajectorySequenceBuilder {
        pushPath()

        val turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(lastPose!!.heading, 0.0, 0.0),
            MotionState(lastPose!!.heading + angle, 0.0, 0.0),
            maxAngularVelocity,
            maxAngularAcceleration
        )

        sequenceSegments.add(TurnSegment(lastPose, angle, turnProfile, mutableListOf()))

        lastPose = Pose2d(
            lastPose!!.x, lastPose!!.y,
            Angle.norm(lastPose!!.heading + angle)
        )

        currentDuration = currentDuration?.plus(turnProfile.duration())

        return this
    }

    fun waitSeconds(seconds: Double): TrajectorySequenceBuilder {
        pushPath()

        sequenceSegments.add(WaitSegment(lastPose, seconds, mutableListOf()))

        currentDuration = currentDuration?.plus(seconds)

        return this
    }

    fun addTrajectory(trajectory: Trajectory): TrajectorySequenceBuilder {
        pushPath()

        sequenceSegments.add(TrajectorySegment(trajectory))

        return this
    }

    fun build(): TrajectorySequence {
        pushPath()

        val globalMarkers = convertMarkersToGlobal(
            sequenceSegments,
            temporalMarkers,
            displacementMarkers,
            spatialMarkers
        )

        return TrajectorySequence(projectGlobalMarkersToLocalSegments(globalMarkers, sequenceSegments))
    }

    private fun pushPath() {
        if (currentTrajectoryBuilder != null) {
            val builtTrajectory = currentTrajectoryBuilder!!.build()
            sequenceSegments.add(TrajectorySegment(builtTrajectory))
        }

        currentTrajectoryBuilder = null
    }

    private fun newPath() {
        if (currentTrajectoryBuilder != null) {
            pushPath()
        }

        lastTrajectoryDuration = 0.0
        lastTrajectoryDisplacement = 0.0

        val tangent: Double = if (setAbsoluteTangent!!) {
            absoluteTangent!!
        } else {
            Angle.norm(lastPose!!.heading + tangentOffset!!)
        }

        currentTrajectoryBuilder = TrajectoryBuilder(
            lastPose!!,
            tangent,
            currentVelocityConstraint!!,
            currentAccelerationConstraint!!,
            resolution
        )
    }

    private fun convertMarkersToGlobal(
        sequenceSegments: MutableList<SequenceSegment>,
        temporalMarkers: MutableList<TemporalMarker>,
        displacementMarkers: MutableList<DisplacementMarker>,
        spatialMarkers: MutableList<SpatialMarker>
    ): MutableList<TrajectoryMarker> {
        val trajectoryMarkers = mutableListOf<TrajectoryMarker>()

        // Convert temporal markers.
        for (temporalMarker in temporalMarkers) {
            trajectoryMarkers.add(
                TrajectoryMarker(temporalMarker.producer.produce(currentDuration!!), temporalMarker.callback)
            )
        }

        // Convert displacement markers.
        for (displacementMarker in displacementMarkers) {
            val time = displacementToTime(
                sequenceSegments,
                displacementMarker.producer.produce(currentDisplacement!!)
            )

            trajectoryMarkers.add(
                TrajectoryMarker(time, displacementMarker.callback)
            )
        }

        // Convert spatial markers.
        for (spatialMarker in spatialMarkers) {
            trajectoryMarkers.add(
                TrajectoryMarker(
                    pointToTime(sequenceSegments, spatialMarker.point),
                        spatialMarker.callback
                )
            )
        }

        return trajectoryMarkers
    }

    // Taken directly from Roadrunner's TrajectoryGenerator.displacementToTime() since it's private.
    // Assumes that the profile position is monotonic increasing.
    private fun motionProfileDisplacementToTime(motionProfile: MotionProfile, displacement: Double): Double {
        var lowTime = 0.0
        var highTime = motionProfile.duration()

        while (abs(lowTime - highTime) >= 1e-6) {
            val midTime = 0.5 * (lowTime + highTime)

            if (motionProfile[midTime].x > displacement) {
                highTime = midTime
            } else {
                lowTime = midTime
            }
        }

        return 0.5 * (lowTime + highTime)
    }

    private fun projectGlobalMarkersToLocalSegments(
        markers: MutableList<TrajectoryMarker>,
        sequenceSegments: MutableList<SequenceSegment>
    ): MutableList<SequenceSegment> {
        if (sequenceSegments.isEmpty()) {
            return mutableListOf()
        }

        var totalSequenceDuration = 0.0

        for (sequenceSegment in sequenceSegments) {
            totalSequenceDuration += sequenceSegment.getDuration()!!
        }

        for (marker in markers) {
            var segment: SequenceSegment? = null
            var segmentIndex = 0
            var segmentOffsetTime = 0.0
            var currentTime = 0.0

            for (i in 0 until sequenceSegments.size) {
                val thisSegment = sequenceSegments[i]
                val markerTime = min(marker.time, totalSequenceDuration)

                if (currentTime + thisSegment.getDuration()!! >= markerTime) {
                    segment = thisSegment
                    segmentIndex = i
                    segmentOffsetTime = markerTime - currentTime
                    break
                } else {
                    currentTime += thisSegment.getDuration()!!
                }
            }

            var newSegment: SequenceSegment? = null

            when (segment) {
                is WaitSegment -> {
                    val newMarkers = segment.getMarkers()
                    newMarkers?.addAll(sequenceSegments[segmentIndex].getMarkers()!!)
                    newMarkers?.add(TrajectoryMarker(segmentOffsetTime, marker.callback))

                    val thisSegment = segment as WaitSegment
                    newSegment = WaitSegment(thisSegment.getStartPose(), thisSegment.getDuration()!!, newMarkers)
                }
                is TurnSegment -> {
                    val newMarkers = segment.getMarkers()
                    newMarkers?.addAll(sequenceSegments[segmentIndex].getMarkers()!!)
                    newMarkers?.add(TrajectoryMarker(segmentOffsetTime, marker.callback))

                    val thisSegment = segment as TurnSegment
                    newSegment = TurnSegment(thisSegment.getStartPose(), thisSegment.getTotalRotation(), thisSegment.getMotionProfile(), newMarkers)
                }
                is TrajectorySegment -> {
                    val thisSegment = segment as TrajectorySegment

                    val newMarkers = thisSegment.getTrajectory()?.markers as MutableList<TrajectoryMarker>
                    newMarkers.add(TrajectoryMarker(segmentOffsetTime, marker.callback))

                    newSegment = TrajectorySegment(
                        Trajectory(
                            thisSegment.getTrajectory()!!.path,
                            thisSegment.getTrajectory()!!.profile,
                            newMarkers
                        )
                    )
                }
            }

            sequenceSegments[segmentIndex] = newSegment!!
        }

        return sequenceSegments
    }

    private fun displacementToTime(sequenceSegments: MutableList<SequenceSegment>, displacement: Double): Double {
        var currentTime = 0.0
        var currentDisplacement = 0.0

        for (sequenceSegment in sequenceSegments) {
            if (sequenceSegment is TrajectorySegment) {
                val thisSegment = sequenceSegment as TrajectorySegment

                val segmentLength = thisSegment.getTrajectory()!!.path.length() // in

                if (currentDisplacement + segmentLength > displacement) {
                    val target = displacement - currentDisplacement
                    val timeInSegment = motionProfileDisplacementToTime(thisSegment.getTrajectory()!!.profile, target)

                    return currentTime + timeInSegment
                } else {
                    currentDisplacement += segmentLength
                    currentTime += thisSegment.getTrajectory()!!.duration()
                }
            } else {
                currentTime += sequenceSegment.getDuration()!!
            }
        }

        // Default return, target displacement is greater than total trajectory displacement.
        return 0.0
    }

    private fun pointToTime(sequenceSegments: MutableList<SequenceSegment>, point: Vector2d): Double {
        data class ComparingPoints(
            var distanceToPoint: Double,
            var totalDisplacement: Double,
            var thisPathDisplacement: Double
        )

        val projectedPoints = mutableListOf<ComparingPoints>()

        for (sequenceSegment in sequenceSegments) {
            if (sequenceSegment !is TrajectorySegment) {
                continue
            }

            val displacement = sequenceSegment.getTrajectory()!!.path.project(point, 0.25)
            val projectedPoint = sequenceSegment.getTrajectory()!!.path[displacement].vec()
            val distanceToPoint = point.minus(projectedPoint).norm()

            var totalDisplacement = 0.0

            for (comparingPoint in projectedPoints) {
                totalDisplacement += comparingPoint.totalDisplacement
            }

            totalDisplacement += displacement

            projectedPoints.add(ComparingPoints(distanceToPoint, displacement, totalDisplacement))
        }

        val closestPoint = projectedPoints.minByOrNull { it.distanceToPoint }

        return displacementToTime(sequenceSegments, closestPoint!!.thisPathDisplacement)
    }

    private interface AddPathCallback {
        fun run()
    }
}
