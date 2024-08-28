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

using System.Numerics;

namespace Jitter.LinearMath
{
    /// <summary>
    /// Bounding Box defined through min and max vectors. Member
    /// of the math namespace, so every method has it's 'by reference'
    /// equivalent to speed up time critical math operations.
    /// </summary>
    public struct JBBox
    {
        /// <summary>
        /// Containment type used within the <see cref="JBBox"/> structure.
        /// </summary>
        public enum ContainmentType
        {
            /// <summary>
            /// The objects don't intersect.
            /// </summary>
            Disjoint,

            /// <summary>
            /// One object is within the other.
            /// </summary>
            Contains,

            /// <summary>
            /// The two objects intersect.
            /// </summary>
            Intersects
        }

        /// <summary>
        /// The maximum point of the box.
        /// </summary>
        public Vector3 Min;

        /// <summary>
        /// The minimum point of the box.
        /// </summary>
        public Vector3 Max;

        /// <summary>
        /// Returns the largest box possible.
        /// </summary>
        public static readonly JBBox LargeBox = new JBBox(new(float.MinValue), new(float.MaxValue));

        /// <summary>
        /// Returns the smalltest box possible.
        /// </summary>
        public static readonly JBBox SmallBox = new JBBox(new(float.MaxValue), new(float.MinValue));

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="min">The minimum point of the box.</param>
        /// <param name="max">The maximum point of the box.</param>
        public JBBox(Vector3 min, Vector3 max)
        {
            Min = min;
            Max = max;
        }

        /// <summary>
        /// Transforms the bounding box into the space given by orientation and position.
        /// </summary>
        /// <param name="position"></param>
        /// <param name="orientation"></param>
        internal void InverseTransform(ref Vector3 position, ref JMatrix orientation)
        {
            Max -= position;
            Min -= position;

            var center = Max + Min;
            center.X *= 0.5f; center.Y *= 0.5f; center.Z *= 0.5f;

            var halfExtents = Max - Min;
            halfExtents.X *= 0.5f; halfExtents.Y *= 0.5f; halfExtents.Z *= 0.5f;

            center = JVectorExtensions.TransposedTransform(center, orientation);

            var abs = orientation.Absolute();
            halfExtents = JVectorExtensions.TransposedTransform(halfExtents, abs);

            Max = center + halfExtents;
            Min = center - halfExtents;
        }

        public void Transform(ref JMatrix orientation)
        {
            var halfExtents = 0.5f * (Max - Min);
            var center = 0.5f * (Max + Min);

            center = JVectorExtensions.Transform(center, orientation);

            var abs = orientation.Absolute();
            halfExtents = JVectorExtensions.Transform(halfExtents, abs);

            Max = center + halfExtents;
            Min = center - halfExtents;
        }

        /// <summary>
        /// Checks whether a point is inside, outside or intersecting
        /// a point.
        /// </summary>
        /// <returns>The ContainmentType of the point.</returns>
        private readonly bool Intersect1D(float start, float dir, float min, float max, ref float enter,ref float exit)
        {
            if (dir * dir < JMath.Epsilon * JMath.Epsilon) return start >= min && start <= max;

            var t0 = (min - start) / dir;
            var t1 = (max - start) / dir;

            if (t0 > t1)
                (t0, t1) = (t1, t0);

            if (t0 > exit || t1 < enter)
                return false;

            if (t0 > enter)
                enter = t0;
            if (t1 < exit)
                exit = t1;
            return true;
        }


        public readonly bool SegmentIntersect(Vector3 origin, Vector3 direction)
        {
            float enter = 0.0f, exit = 1.0f;

            if (!Intersect1D(origin.X, direction.X, Min.X, Max.X,ref enter,ref exit))
                return false;

            if (!Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, ref enter, ref exit))
                return false;

            if (!Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z,ref enter,ref exit))
                return false;

            return true;
        }

        public readonly bool RayIntersect(Vector3 origin, Vector3 direction)
        {
            float enter = 0.0f, exit = float.MaxValue;

            if (!Intersect1D(origin.X, direction.X, Min.X, Max.X, ref enter, ref exit))
                return false;

            if (!Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, ref enter, ref exit))
                return false;

            if (!Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, ref enter, ref exit))
                return false;

            return true;
        }

        /// <summary>
        /// Checks whether a point is inside, outside or intersecting
        /// a point.
        /// </summary>
        /// <param name="point">A point in space.</param>
        /// <returns>The ContainmentType of the point.</returns>
        public readonly ContainmentType Contains(Vector3 point)
        {
            return Min.X <= point.X && point.X <= Max.X &&
                   Min.Y <= point.Y && point.Y <= Max.Y &&
                   Min.Z <= point.Z && point.Z <= Max.Z
                 ? ContainmentType.Contains
                 : ContainmentType.Disjoint;
        }

        /// <summary>
        /// Retrieves the 8 corners of the box.
        /// </summary>
        /// <returns>An array of 8 Vector3 entries.</returns>
        public readonly void GetCorners(Vector3[] corners)
        {
            corners[0] = new(Min.X, Max.Y, Max.Z);
            corners[1] = new(Max.X, Max.Y, Max.Z);
            corners[2] = new(Max.X, Min.Y, Max.Z);
            corners[3] = new(Min.X, Min.Y, Max.Z);
            corners[4] = new(Min.X, Max.Y, Min.Z);
            corners[5] = new(Max.X, Max.Y, Min.Z);
            corners[6] = new(Max.X, Min.Y, Min.Z);
            corners[7] = new(Min.X, Min.Y, Min.Z);
        }

        /// <summary>
        /// Expand this box to include the given point
        /// </summary>
        /// <param name="point"></param>
        public void AddPoint(Vector3 point)
        {
            Max = Vector3.Max(Max, point);
            Min = Vector3.Min(Min, point);
        }

        /// <summary>
        /// Expands a bounding box with the volume 0 by all points
        /// given.
        /// </summary>
        /// <param name="points">A array of Vector3.</param>
        /// <returns>The resulting bounding box containing all points.</returns>
        public static JBBox CreateFromPoints(Vector3[] points)
        {
            var vector3 = new Vector3(float.MaxValue);
            var vector2 = new Vector3(float.MinValue);

            for (var i = 0; i < points.Length; i++)
            {
                vector3 = Vector3.Min(vector3, points[i]);
                vector2 = Vector3.Max(vector2, points[i]);
            }
            return new(vector3, vector2);
        }

        /// <summary>
        /// Checks whether another bounding box is inside, outside or intersecting
        /// this box. 
        /// </summary>
        /// <param name="box">The other bounding box to check.</param>
        /// <returns>The ContainmentType of the box.</returns>
        public readonly ContainmentType Contains(JBBox box)
        {
            return Contains(ref box);
        }

        /// <summary>
        /// Checks whether another bounding box is inside, outside or intersecting
        /// this box. 
        /// </summary>
        /// <param name="box">The other bounding box to check.</param>
        /// <returns>The ContainmentType of the box.</returns>
        public readonly ContainmentType Contains(ref JBBox box)
        {
            var result = ContainmentType.Disjoint;
            if (Max.X >= box.Min.X && Min.X <= box.Max.X && Max.Y >= box.Min.Y && Min.Y <= box.Max.Y && Max.Z >= box.Min.Z && Min.Z <= box.Max.Z)
            {
                result = Min.X <= box.Min.X && box.Max.X <= Max.X && Min.Y <= box.Min.Y && box.Max.Y <= Max.Y && Min.Z <= box.Min.Z && box.Max.Z <= Max.Z ? ContainmentType.Contains : ContainmentType.Intersects;
            }

            return result;
        }

        /// <summary>
        /// Creates a new box containing the two given ones.
        /// </summary>
        /// <param name="original">First box.</param>
        /// <param name="additional">Second box.</param>
        /// <returns>A JBBox containing the two given boxes.</returns>
        public static JBBox CreateMerged(JBBox original, JBBox additional)
        {
            CreateMerged(ref original, ref additional, out var result);
            return result;
        }

        /// <summary>
        /// Creates a new box containing the two given ones.
        /// </summary>
        /// <param name="original">First box.</param>
        /// <param name="additional">Second box.</param>
        /// <param name="result">A JBBox containing the two given boxes.</param>
        public static void CreateMerged(ref JBBox original, ref JBBox additional, out JBBox result)
        {
            var vector2 = Vector3.Min(original.Min, additional.Min);
            var vector = Vector3.Max(original.Max, additional.Max);
            result.Min = vector2;
            result.Max = vector;
        }

        public readonly Vector3 Center => (Min + Max) * 0.5f;

        internal readonly float Perimeter =>
            2.0f * ((Max.X - Min.X) * (Max.Y - Min.Y) +
                    (Max.X - Min.X) * (Max.Z - Min.Z) +
                    (Max.Z - Min.Z) * (Max.Y - Min.Y));
    }
}
