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
        DhParam(0, 0, 10, 0)
    ).links()

    @Test
    fun `test fk to ik`() {
        (10..20 step 1).map { targetPos ->
            val target = FrameTransformation.fromTranslation(targetPos, 5, 0)

            val resultAngles = ik.solveChain(
                planarLinks,
                planarLinks.map { 0.0 }.toImmutableList(),
                target
            )

            val resultTarget = fk.solveChain(planarLinks, resultAngles)

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
}
