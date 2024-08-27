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

namespace Jitter.LinearMath
{
    public static class JVectorExtensions
    {
        private const float ZeroEpsilonSq = JMath.Epsilon * JMath.Epsilon;

        /// <summary>
        /// Checks if the length of the vector is nearly zero.
        /// </summary>
        /// <returns>Returns true if the vector is nearly zero, otherwise false.</returns>
        public static bool IsNearlyZero(this JVector vector)
        {
            return vector.LengthSquared() < ZeroEpsilonSq;
        }

        /// <summary>
        /// Transforms a vector by the given matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <returns>The transformed vector.</returns>
        public static JVector Transform(JVector position, JMatrix matrix)
        {
            var num0 = position.X * matrix.M11 + position.Y * matrix.M21 + position.Z * matrix.M31;
            var num1 = position.X * matrix.M12 + position.Y * matrix.M22 + position.Z * matrix.M32;
            var num2 = position.X * matrix.M13 + position.Y * matrix.M23 + position.Z * matrix.M33;

            return new JVector
            {
                X = num0,
                Y = num1,
                Z = num2
            };
        }

        public static JVector TransposedTransform(JVector position, JMatrix matrix)
        {
            var num0 = position.X * matrix.M11 + position.Y * matrix.M12 + position.Z * matrix.M13;
            var num1 = position.X * matrix.M21 + position.Y * matrix.M22 + position.Z * matrix.M23;
            var num2 = position.X * matrix.M31 + position.Y * matrix.M32 + position.Z * matrix.M33;

            return new JVector
            {
                X = num0,
                Y = num1,
                Z = num2,
            };
        }
    }

    /// <summary>
    /// A vector structure. Member of the math 
    /// namespace, so every method has it's 'by reference' equivalent
    /// to speed up time critical math operations.
    /// </summary>
    public struct JVector
    {
        /// <summary>The X component of the vector.</summary>
        public float X;
        /// <summary>The Y component of the vector.</summary>
        public float Y;
        /// <summary>The Z component of the vector.</summary>
        public float Z;

        /// <summary>
        /// A vector with components (0,0,0);
        /// </summary>
        public static readonly JVector Zero;
        /// <summary>
        /// A vector with components (1,0,0);
        /// </summary>
        public static readonly JVector Left;
        /// <summary>
        /// A vector with components (-1,0,0);
        /// </summary>
        public static readonly JVector Right;
        /// <summary>
        /// A vector with components (0,1,0);
        /// </summary>
        public static readonly JVector Up;
        /// <summary>
        /// A vector with components (0,-1,0);
        /// </summary>
        public static readonly JVector Down;
        /// <summary>
        /// A vector with components (0,0,1);
        /// </summary>
        public static readonly JVector Backward;
        /// <summary>
        /// A vector with components (0,0,-1);
        /// </summary>
        public static readonly JVector Forward;
        /// <summary>
        /// A vector with components (1,1,1);
        /// </summary>
        public static readonly JVector One;

        static JVector()
        {
            One = new JVector(1, 1, 1);
            Zero = new JVector(0, 0, 0);
            Left = new JVector(1, 0, 0);
            Right = new JVector(-1, 0, 0);
            Up = new JVector(0, 1, 0);
            Down = new JVector(0, -1, 0);
            Backward = new JVector(0, 0, 1);
            Forward = new JVector(0, 0, -1);
        }

        /// <summary>
        /// Constructor initializing a new instance of the structure
        /// </summary>
        /// <param name="x">The X component of the vector.</param>
        /// <param name="y">The Y component of the vector.</param>
        /// <param name="z">The Z component of the vector.</param>
        public JVector(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        /// <summary>
        /// Constructor initializing a new instance of the structure
        /// </summary>
        /// <param name="xyz">All components of the vector are set to xyz</param>
        public JVector(float xyz)
        {
            X = xyz;
            Y = xyz;
            Z = xyz;
        }

        /// <summary>
        /// Builds a string from the JVector.
        /// </summary>
        /// <returns>A string containing all three components.</returns>
        public override string ToString()
        {
            return $"X={X} Y={Y} Z={Z}";
        }

        /// <summary>
        /// Tests if an object is equal to this vector.
        /// </summary>
        /// <param name="obj">The object to test.</param>
        /// <returns>Returns true if they are euqal, otherwise false.</returns>
        public override bool Equals(object obj)
        {
            if (obj is not JVector other)
                return false;

            return X == other.X && Y == other.Y && Z == other.Z;
        }

        /// <summary>
        /// Tests if two JVector are equal.
        /// </summary>
        /// <param name="value1">The first value.</param>
        /// <param name="value2">The second value.</param>
        /// <returns>Returns true if both values are equal, otherwise false.</returns>
        public static bool operator ==(JVector value1, JVector value2)
        {
            return value1.X == value2.X && value1.Y == value2.Y && value1.Z == value2.Z;
        }

        /// <summary>
        /// Tests if two JVector are not equal.
        /// </summary>
        /// <param name="value1">The first value.</param>
        /// <param name="value2">The second value.</param>
        /// <returns>Returns false if both values are equal, otherwise true.</returns>
        public static bool operator !=(JVector value1, JVector value2)
        {
            if (value1.X == value2.X && value1.Y == value2.Y)
            {
                return value1.Z != value2.Z;
            }
            return true;
        }

        /// <summary>
        ///  ets a vector with the minimum x,y and z values of both vectors.
        /// </summary>
        /// <param name="value1">The first value.</param>
        /// <param name="value2">The second value.</param>
        /// <returns>A vector with the minimum x,y and z values of both vectors.</returns>
        public static JVector Min(JVector value1, JVector value2)
        {
            JVector result;
            result.X = value1.X < value2.X ? value1.X : value2.X;
            result.Y = value1.Y < value2.Y ? value1.Y : value2.Y;
            result.Z = value1.Z < value2.Z ? value1.Z : value2.Z;
            return result;
        }

        /// <summary>
        /// Gets a vector with the maximum x,y and z values of both vectors.
        /// </summary>
        /// <param name="value1">The first value.</param>
        /// <param name="value2">The second value.</param>
        /// <returns>A vector with the maximum x,y and z values of both vectors.</returns>
        public static JVector Max(JVector value1, JVector value2)
        {
            JVector result;
            result.X = value1.X > value2.X ? value1.X : value2.X;
            result.Y = value1.Y > value2.Y ? value1.Y : value2.Y;
            result.Z = value1.Z > value2.Z ? value1.Z : value2.Z;
            return result;
        }

        /// <summary>
        /// Calculates the dot product of both vectors.
        /// </summary>
        /// <param name="vector1">The first vector.</param>
        /// <param name="vector2">The second vector.</param>
        /// <returns>Returns the dot product of both vectors.</returns>
        public static float Dot(JVector vector1, JVector vector2)
        {
            return vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z;
        }

        /// <summary>
        /// The cross product of two vectors.
        /// </summary>
        /// <param name="vector1">The first vector.</param>
        /// <param name="vector2">The second vector.</param>
        /// <param name="result">The cross product of both vectors.</param>
        public static JVector Cross(JVector vector1, JVector vector2)
        {
            var num3 = vector1.Y * vector2.Z - vector1.Z * vector2.Y;
            var num2 = vector1.Z * vector2.X - vector1.X * vector2.Z;
            var num = vector1.X * vector2.Y - vector1.Y * vector2.X;

            var result = new JVector
            {
                X = num3,
                Y = num2,
                Z = num,
            };

            return result;
        }

        /// <summary>
        /// Gets the hashcode of the vector.
        /// </summary>
        /// <returns>Returns the hashcode of the vector.</returns>
        public override int GetHashCode()
        {
            return HashCode.Combine(X, Y, Z);
        }

        /// <summary>
        /// Normalizes the given vector.
        /// </summary>
        /// <param name="value">The vector which should be normalized.</param>
        /// <returns>A normalized vector.</returns>
        public static JVector Normalize(JVector value)
        {
            return value * (1 / value.Length());
        }

        /// <summary>
        /// Gets the squared length of the vector.
        /// </summary>
        /// <returns>Returns the squared length of the vector.</returns>
        public float LengthSquared()
        {
            return X * X + Y * Y + Z * Z;
        }

        /// <summary>
        /// Gets the length of the vector.
        /// </summary>
        /// <returns>Returns the length of the vector.</returns>
        public float Length()
        {
            var num = X * X + Y * Y + Z * Z;
            return MathF.Sqrt(num);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="value1"></param>
        /// <returns></returns>
        public static JVector operator -(JVector value1)
        {
            return new JVector(-value1.X, -value1.Y, -value1.Z);
        }

        /// <summary>
        /// Multiplies a vector by a scale factor.
        /// </summary>
        /// <param name="value1">The vector to scale.</param>
        /// <param name="value2">The scale factor.</param>
        /// <returns>Returns the scaled vector.</returns>
        public static JVector operator *(JVector value1, float value2)
        {
            JVector result;
            result.X = value1.X * value2;
            result.Y = value1.Y * value2;
            result.Z = value1.Z * value2;
            return result;
        }

        /// <summary>
        /// Multiplies a vector by a scale factor.
        /// </summary>
        /// <param name="value2">The vector to scale.</param>
        /// <param name="value1">The scale factor.</param>
        /// <returns>Returns the scaled vector.</returns>
        public static JVector operator *(float value1, JVector value2)
        {
            var result = value2 * value1;
            return result;
        }

        /// <summary>
        /// Subtracts two vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The difference of both vectors.</returns>
        public static JVector operator -(JVector value1, JVector value2)
        {
            return new JVector(
                value1.X - value2.X,
                value1.Y - value2.Y,
                value1.Z - value2.Z
            );
        }

        /// <summary>
        /// Adds two vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The sum of both vectors.</returns>
        public static JVector operator +(JVector value1, JVector value2)
        {
            return new JVector(
                value1.X + value2.X,
                value1.Y + value2.Y,
                value1.Z + value2.Z
            );
        }
    }
}
