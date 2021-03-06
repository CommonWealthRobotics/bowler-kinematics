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
package com.neuronrobotics.kinematicschef.dhparam

import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DhParam
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.params.ParameterizedTest
import org.junit.jupiter.params.provider.ValueSource
import kotlin.math.sqrt
import kotlin.random.Random

internal class DhParamTest {

    @ParameterizedTest
    @ValueSource(ints = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
    fun `test length on d`(testValue: Int) {
        assertEquals(testValue.toDouble(), DhParam(testValue, 0, 0, 0).length)
    }

    @ParameterizedTest
    @ValueSource(ints = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
    fun `test length on r`(testValue: Int) {
        assertEquals(testValue.toDouble(), DhParam(0, 0, testValue, 0).length)
    }

    @ParameterizedTest
    @ValueSource(ints = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
    fun `test length on d and r with random theta and alpha`(testValue: Int) {
        assertEquals(
            sqrt(2.0) * testValue,
            DhParam(testValue, Random.nextDouble(90.0), testValue, Random.nextDouble(90.0)).length,
            1e-10
        )
    }
}
