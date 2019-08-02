/*
 * This file is part of kinematics-chef.
 *
 * kinematics-chef is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * kinematics-chef is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with kinematics-chef.  If not, see <https://www.gnu.org/licenses/>.
 */
package com.neuronrobotics.kinematicschef.solver

import com.google.common.collect.ImmutableList
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DefaultLink
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.Link
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.toFrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.FrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.InverseKinematicsSolver
import com.neuronrobotics.kinematicschef.util.modulus
import org.apache.commons.math3.geometry.euclidean.threed.Plane
import org.apache.commons.math3.geometry.euclidean.threed.Rotation
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.geometry.euclidean.twod.Line
import org.octogonapus.ktguava.collections.toImmutableList
import java.lang.Math.toDegrees
import kotlin.math.acos
import kotlin.math.pow
import kotlin.math.sign

class CCDSolver : InverseKinematicsSolver {

    override fun solveChain(
        links: ImmutableList<Link>,
        currentJointAngles: ImmutableList<Double>,
        targetFrameTransform: FrameTransformation
    ) = solveChain(links, currentJointAngles, targetFrameTransform, 1e-3, 10000)

    fun solveChain(
        links: ImmutableList<Link>,
        currentJointAngles: ImmutableList<Double>,
        targetFrameTransform: FrameTransformation,
        tolerance: Double,
        maxIterations: Int
    ): ImmutableList<Double> {
        val mutLinks = links.toMutableList()

        fun calcDelta(tip: FrameTransformation): Double = Vector3D(
            targetFrameTransform.translationX,
            targetFrameTransform.translationY,
            targetFrameTransform.translationZ
        ).subtract(Vector3D(tip.translationX, tip.translationY, tip.translationZ)).norm

        var iter = 0
        var delta = calcDelta(mutLinks.tip())

        while (iter < maxIterations && delta > tolerance) {
            for (i in links.size - 1 downTo 0) {
                val theta = calcTheta(targetFrameTransform, mutLinks, i)
                mutLinks[i] = mutLinks[i].addTheta(toDegrees(theta))
                println(mutLinks.map { it.dhParam.theta })

                val newDelta = calcDelta(mutLinks.tip())
                if (newDelta > delta) {
                    mutLinks[i] = mutLinks[i].addTheta(-2 * toDegrees(theta))
                    println(
                        """
                        Delta increased!
                        iter: $iter
                        linkIndex: $i
                        theta: ${toDegrees(theta)}
                        """.trimIndent()
                    )
                }
            }

            iter++
            delta = calcDelta(mutLinks.tip())
        }

        return mutLinks.zip(links)
            .map { (mutLink, link) -> mutLink.dhParam.theta - link.dhParam.theta }
            .toImmutableList()
    }

    private fun calcTheta(
        target: FrameTransformation,
        chain: List<Link>,
        currentIndex: Int
    ): Double {
        val tip = chain.tip()
        val currentFt = chain.subList(0, currentIndex).tip()
        val rotation = Rotation(currentFt.rotation.transpose().array, 1e-10)

        val targetVector = Vector3D(
            target.translationX,
            target.translationY,
            target.translationZ
        )

        val tipVector = Vector3D(
            tip.translationX,
            tip.translationY,
            tip.translationZ
        )

        val baseVector = Vector3D(
            currentFt.translationX,
            currentFt.translationY,
            currentFt.translationZ
        )

        val linkPlane = Plane(Vector3D(0.0, 0.0, 1.0), 1e-10)
            .translate(baseVector)
            .rotate(baseVector, rotation)

        val projectedTarget = projectOntoPlane(baseVector, targetVector, linkPlane)
        val projectedTip = projectOntoPlane(baseVector, tipVector, linkPlane)

        val pt = linkPlane.toSubSpace(projectedTarget)
        val pe = linkPlane.toSubSpace(projectedTip)
        val pc = linkPlane.toSubSpace(baseVector)

        val c = pt.subtract(pe).norm
        val a = pt.subtract(pc).norm
        val b = pe.subtract(pc).norm

        val sign = -sign(Line(pc, pt, 1e-10).getOffset(pe))

        return if (a == 0.0 || b == 0.0) {
            0.0
        } else {
            sign * acos((c.pow(2) - a.pow(2) - b.pow(2)) / -(2 * a * b))
                .let { if (it.isNaN()) 0.0 else it }
        }
    }

    private fun projectOntoPlane(base: Vector3D, toProject: Vector3D, plane: Plane) =
        toProject.subtract(
            plane.normal.scalarMultiply(
                toProject.subtract(base)
                    .dotProduct(plane.normal)
            )
        )

    private fun List<Link>.tip(): FrameTransformation = map { it.dhParam }.toFrameTransformation()

    private fun Link.addTheta(theta: Double): Link = DefaultLink(
        type,
        dhParam.copy(theta = (dhParam.theta + theta).modulus(360)),
        jointLimits,
        inertialStateEstimator
    )
}
