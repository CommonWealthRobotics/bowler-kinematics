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

import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DhParam
import com.neuronrobotics.bowlerkernel.kinematics.motion.FrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.approxEquals
import com.neuronrobotics.kinematicschef.GeneralForwardKinematicsSolver
import com.neuronrobotics.kinematicschef.TestUtil.hephaestusArmLinks
import com.neuronrobotics.kinematicschef.links
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.octogonapus.ktguava.collections.toImmutableList

internal class CCDSolverTest {

    private val fk = GeneralForwardKinematicsSolver()
    private val ik = CCDSolver()

    private val planarLinks = listOf(
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0),
        DhParam(0, 0, 10, 0)
    ).links()

    @Test
    fun `test ik with planar`() {
        (-100..100 step 1).map { targetPos ->
            val links = planarLinks

            // Need a small offset on y
            val target = FrameTransformation.fromTranslation(targetPos, 50, 0)

            val resultAngles = ik.solveChain(
                links,
                links.map { 0.0 }.toImmutableList(),
                target
            )

            val resultTarget = fk.solveChain(links, resultAngles)

            assertTrue(
                target.translation.approxEquals(resultTarget.translation, 0.1)
            ) {
                """
                Target:
                ${target.translation.array.joinToString { it.joinToString() }}
                Result:
                ${resultTarget.translation.array.joinToString { it.joinToString() }}
                Result angles:
                $resultAngles
                """.trimIndent()
            }
        }
    }

    @Test
    fun `test with 3001 arm`() {
        (-200..200 step 1).map { targetPos ->
            if (targetPos !in listOf<Int>(0)) {
                val tolerance = 1e-3
                val links = hephaestusArmLinks
                val target = FrameTransformation.fromTranslation(targetPos, 0, 0)

                val resultAngles = ik.solveChain(
                    links,
                    links.map { 0.0 }.toImmutableList(),
                    target,
                    tolerance,
                    10000
                )

                val resultTarget = fk.solveChain(links, resultAngles)

                println("${target.translation.array.joinToString { it.joinToString() }} $resultAngles")

                assertTrue(
                    target.translation.approxEquals(resultTarget.translation, tolerance)
                ) {
                    """
                    Target:
                    ${target.translation.array.joinToString { it.joinToString() }}
                    Result:
                    ${resultTarget.translation.array.joinToString { it.joinToString() }}
                    Result angles:
                    $resultAngles
                    """.trimIndent()
                }
            }
        }
    }
}
