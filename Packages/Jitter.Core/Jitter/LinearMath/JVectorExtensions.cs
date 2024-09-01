/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

using System;
using System.Numerics;

namespace Jitter.LinearMath
{
    public static class JVectorExtensions
    {
        /// <summary>
        /// A vector with components (0,0,0);
        /// </summary>
        public static readonly Vector3 Zero = new(0);

        /// <summary>
        /// A vector with components (1,0,0);
        /// </summary>
        public static readonly Vector3 Left = new(1, 0, 0);

        /// <summary>
        /// A vector with components (-1,0,0);
        /// </summary>
        public static readonly Vector3 Right = -Left;

        /// <summary>
        /// A vector with components (0,1,0);
        /// </summary>
        public static readonly Vector3 Up = new(0, 1, 0);

        /// <summary>
        /// A vector with components (0,-1,0);
        /// </summary>
        public static readonly Vector3 Down = -Up;

        /// <summary>
        /// A vector with components (0,0,1);
        /// </summary>
        public static readonly Vector3 Backward = new(0, 0, 1);
        /// <summary>
        /// A vector with components (0,0,-1);
        /// </summary>
        public static readonly Vector3 Forward = -Backward;

        /// <summary>
        /// A vector with components (1,1,1);
        /// </summary>
        public static readonly Vector3 One = new(1);

        /// <summary>
        /// Checks if the length of the vector is nearly zero.
        /// </summary>
        /// <returns>Returns true if the vector is nearly zero, otherwise false.</returns>
        public static bool IsNearlyZero(this Vector3 vector)
        {
            const float ZeroEpsilonSq = JMath.Epsilon * JMath.Epsilon;
            return vector.LengthSquared() < ZeroEpsilonSq;
        }

        /// <summary>
        /// Transforms a vector by the given matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <returns>The transformed vector.</returns>
        public static Vector3 Transform(this Vector3 position, in JMatrix matrix)
        {
            var num0 = Vector3.Dot(position, matrix.Col0);
            var num1 = Vector3.Dot(position, matrix.Col1);
            var num2 = Vector3.Dot(position, matrix.Col2);

            return new(num0, num1, num2);
        }

        public static Vector3 TransposedTransform(this Vector3 position, in JMatrix matrix)
        {
            var num0 = Vector3.Dot(position, matrix.Row0);
            var num1 = Vector3.Dot(position, matrix.Row1);
            var num2 = Vector3.Dot(position, matrix.Row2);

            //var num0x = position.X * matrix.M11 + position.Y * matrix.M12 + position.Z * matrix.M13;
            //var num1x = position.X * matrix.M21 + position.Y * matrix.M22 + position.Z * matrix.M23;
            //var num2x = position.X * matrix.M31 + position.Y * matrix.M32 + position.Z * matrix.M33;
            //Debug.Assert(Math.Abs(num0 - num0x) < float.Epsilon);

            return new(num0, num1, num2);
        }

        public static float Element(this Vector3 vector, int axis)
        {
            return axis switch
            {
                0 => vector.X,
                1 => vector.Y,
                2 => vector.Z,
                _ => throw new ArgumentOutOfRangeException(nameof(axis), axis, null)
            };
        }
    }
}
