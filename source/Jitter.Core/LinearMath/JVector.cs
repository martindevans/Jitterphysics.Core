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
using UnityEngine;

namespace Jitter.LinearMath
{
    /// <summary>
    /// A vector structure. Member of the math 
    /// namespace, so every method has it's 'by reference' equivalent
    /// to speed up time critical math operations.
    /// </summary>
    public struct JVector
    {
        private const float ZeroEpsilonSq = JMath.Epsilon * JMath.Epsilon;

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
        /// <summary>
        /// A vector with components 
        /// (float.MinValue,float.MinValue,float.MinValue);
        /// </summary>
        public static readonly JVector MinValue;
        /// <summary>
        /// A vector with components 
        /// (float.MaxValue,float.MaxValue,float.MaxValue);
        /// </summary>
        public static readonly JVector MaxValue;

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
            MinValue = new JVector(float.MinValue);
            MaxValue = new JVector(float.MaxValue);
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
        /// Checks if the length of the vector is nearly zero.
        /// </summary>
        /// <returns>Returns true if the vector is nearly zero, otherwise false.</returns>
        public bool IsNearlyZero()
        {
            return LengthSquared() < ZeroEpsilonSq;
        }

        /// <summary>
        /// Transforms a vector by the given matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <returns>The transformed vector.</returns>
        public static JVector Transform(JVector position, JMatrix matrix)
        {
            Transform(ref position, ref matrix, out var result);
            return result;
        }

        /// <summary>
        /// Transforms a vector by the given matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <param name="result">The transformed vector.</param>
        public static void Transform(ref JVector position, ref JMatrix matrix, out JVector result)
        {
            var num0 = position.X * matrix.M11 + position.Y * matrix.M21 + position.Z * matrix.M31;
            var num1 = position.X * matrix.M12 + position.Y * matrix.M22 + position.Z * matrix.M32;
            var num2 = position.X * matrix.M13 + position.Y * matrix.M23 + position.Z * matrix.M33;

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }

        /// <summary>
        /// Transforms a vector by the transposed of the given Matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <param name="result">The transformed vector.</param>
        public static void TransposedTransform(ref JVector position, ref JMatrix matrix, out JVector result)
        {
            var num0 = position.X * matrix.M11 + position.Y * matrix.M12 + position.Z * matrix.M13;
            var num1 = position.X * matrix.M21 + position.Y * matrix.M22 + position.Z * matrix.M23;
            var num2 = position.X * matrix.M31 + position.Y * matrix.M32 + position.Z * matrix.M33;

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
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
        /// Adds to vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The sum of both vectors.</returns>
        public static JVector Add(JVector value1, JVector value2)
        {
            JVector result;
            var num0 = value1.X + value2.X;
            var num1 = value1.Y + value2.Y;
            var num2 = value1.Z + value2.Z;

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
            return result;
        }

        /// <summary>
        /// Subtracts to vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The difference of both vectors.</returns>
        public static JVector Subtract(JVector value1, JVector value2)
        {
            JVector result;
            var num0 = value1.X - value2.X;
            var num1 = value1.Y - value2.Y;
            var num2 = value1.Z - value2.Z;

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
            return result;
        }

        /// <summary>
        /// The cross product of two vectors.
        /// </summary>
        /// <param name="vector1">The first vector.</param>
        /// <param name="vector2">The second vector.</param>
        /// <param name="result">The cross product of both vectors.</param>
        public static JVector Cross(JVector vector1, JVector vector2)
        {
            var result = new JVector();
            var num3 = vector1.Y * vector2.Z - vector1.Z * vector2.Y;
            var num2 = vector1.Z * vector2.X - vector1.X * vector2.Z;
            var num = vector1.X * vector2.Y - vector1.Y * vector2.X;
            result.X = num3;
            result.Y = num2;
            result.Z = num;

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
        /// Inverses the direction of the vector.
        /// </summary>
        public void Negate()
        {
            X = -X;
            Y = -Y;
            Z = -Z;
        }

        /// <summary>
        /// Inverses the direction of a vector.
        /// </summary>
        /// <param name="value">The vector to inverse.</param>
        /// <returns>The negated vector.</returns>
        public static JVector Negate(JVector value)
        {
            Negate(ref value,out var result);
            return result;
        }

        /// <summary>
        /// Inverses the direction of a vector.
        /// </summary>
        /// <param name="value">The vector to inverse.</param>
        /// <param name="result">The negated vector.</param>
        public static void Negate(ref JVector value, out JVector result)
        {
            var num0 = -value.X;
            var num1 = -value.Y;
            var num2 = -value.Z;

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }

        /// <summary>
        /// Normalizes the given vector.
        /// </summary>
        /// <param name="value">The vector which should be normalized.</param>
        /// <returns>A normalized vector.</returns>
        public static JVector Normalize(JVector value)
        {
            Normalize(ref value, out var result);
            return result;
        }

        /// <summary>
        /// Normalizes this vector.
        /// </summary>
        public void Normalize()
        {
            var num2 = X * X + Y * Y + Z * Z;
            var num = 1f / (float)Math.Sqrt(num2);
            X *= num;
            Y *= num;
            Z *= num;
        }

        /// <summary>
        /// Normalizes the given vector.
        /// </summary>
        /// <param name="value">The vector which should be normalized.</param>
        /// <param name="result">A normalized vector.</param>
        public static void Normalize(ref JVector value, out JVector result)
        {
            var num2 = value.X * value.X + value.Y * value.Y + value.Z * value.Z;
            var num = 1f / (float)Math.Sqrt(num2);
            result.X = value.X * num;
            result.Y = value.Y * num;
            result.Z = value.Z * num;
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
        /// Multiply a vector with a factor.
        /// </summary>
        /// <param name="value1">The vector to multiply.</param>
        /// <param name="scaleFactor">The scale factor.</param>
        /// <param name="result">Returns the multiplied vector.</param>
        public static JVector Multiply(JVector value1, float scaleFactor)
        {
            JVector result;
            result.X = value1.X * scaleFactor;
            result.Y = value1.Y * scaleFactor;
            result.Z = value1.Z * scaleFactor;
            return result;
        }

        public static JVector operator -(JVector value1)
        {
            return new JVector(-value1.X, -value1.Y, -value1.Z);
        }

        /// <summary>
        /// Calculates the cross product of two vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>Returns the cross product of both.</returns>
        public static JVector operator %(JVector value1, JVector value2)
        {
            var result = Cross(value1, value2);
            return result;
        }

        /// <summary>
        /// Calculates the dot product of two vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>Returns the dot product of both.</returns>
        public static float operator *(JVector value1, JVector value2)
        {
            return Dot(value1, value2);
        }

        /// <summary>
        /// Multiplies a vector by a scale factor.
        /// </summary>
        /// <param name="value1">The vector to scale.</param>
        /// <param name="value2">The scale factor.</param>
        /// <returns>Returns the scaled vector.</returns>
        public static JVector operator *(JVector value1, float value2)
        {
            var result = Multiply(value1, value2);
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
            var result = Multiply(value2, value1);
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
            var result = Subtract(value1, value2);
            return result;
        }

        /// <summary>
        /// Adds two vectors.
        /// </summary>
        /// <param name="value1">The first vector.</param>
        /// <param name="value2">The second vector.</param>
        /// <returns>The sum of both vectors.</returns>
        public static JVector operator +(JVector value1, JVector value2)
        {
            var result = Add(value1, value2);
            return result;
        }
    }
}
