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
import com.google.common.math.DoubleMath
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DefaultLink
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.Link
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.toFrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.FrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.InverseKinematicsSolver
import com.neuronrobotics.kinematicschef.util.modulus
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.octogonapus.ktguava.collections.toImmutableList
import java.lang.Math.toDegrees
import kotlin.math.acos
import kotlin.math.pow

class CCDSolver : InverseKinematicsSolver {

    override fun solveChain(
        links: ImmutableList<Link>,
        currentJointAngles: ImmutableList<Double>,
        targetFrameTransform: FrameTransformation
    ) = solveChain(links, currentJointAngles, targetFrameTransform, 0.1, 100)

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
                if (i % 2 == 0) {
                    mutLinks[i] = mutLinks[i].addTheta(toDegrees(-theta))
                } else {
                    mutLinks[i] = mutLinks[i].addTheta(toDegrees(theta))
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

        val pt = Vector2D(target.translationX, target.translationY)
        val pe = Vector2D(tip.translationX, tip.translationY)
        val pc = Vector2D(currentFt.translationX, currentFt.translationY)

        val c = pt.subtract(pe).norm
        val a = pt.subtract(pc).norm
        val b = pe.subtract(pc).norm

        // If pe is inline but above pt it needs to be moved down
        val sign = if (pe.y > pt.y && DoubleMath.fuzzyEquals(pe.x, pt.x, 1e-4)) {
            -1
        } else {
            1
        }

        return if (a == 0.0 || b == 0.0) {
            0.0
        } else {
            sign * acos((c.pow(2) - a.pow(2) - b.pow(2)) / -(2 * a * b))
        }
    }

    private fun List<Link>.tip(): FrameTransformation = map { it.dhParam }.toFrameTransformation()

    private fun Link.addTheta(theta: Double): Link = DefaultLink(
        type,
        dhParam.copy(theta = (dhParam.theta + theta).modulus(360)),
        jointLimits,
        inertialStateEstimator
    )
}
