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
        public JVector Min;

        /// <summary>
        /// The minimum point of the box.
        /// </summary>
        public JVector Max;

        /// <summary>
        /// Returns the largest box possible.
        /// </summary>
        public static readonly JBBox LargeBox;

        /// <summary>
        /// Returns the smalltest box possible.
        /// </summary>
        public static readonly JBBox SmallBox;

        static JBBox()
        {
            LargeBox.Min = new JVector(float.MinValue);
            LargeBox.Max = new JVector(float.MaxValue);
            SmallBox.Min = new JVector(float.MaxValue);
            SmallBox.Max = new JVector(float.MinValue);
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="min">The minimum point of the box.</param>
        /// <param name="max">The maximum point of the box.</param>
        public JBBox(JVector min, JVector max)
        {
            Min = min;
            Max = max;
        }

        /// <summary>
        /// Transforms the bounding box into the space given by orientation and position.
        /// </summary>
        /// <param name="position"></param>
        /// <param name="orientation"></param>
        /// <param name="result"></param>
        internal void InverseTransform(ref JVector position, ref JMatrix orientation)
        {
            Max = Max - position;
            Min = Min - position;

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
        private bool Intersect1D(float start, float dir, float min, float max,
                                 ref float enter,ref float exit)
        {
            if (dir * dir < JMath.Epsilon * JMath.Epsilon) return start >= min && start <= max;

            var t0 = (min - start) / dir;
            var t1 = (max - start) / dir;

            if (t0 > t1) { var tmp = t0; t0 = t1; t1 = tmp; }

            if (t0 > exit || t1 < enter) return false;

            if (t0 > enter) enter = t0;
            if (t1 < exit) exit = t1;
            return true;
        }


        public bool SegmentIntersect(ref JVector origin,ref JVector direction)
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

        public bool RayIntersect(ref JVector origin, ref JVector direction)
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

        public bool SegmentIntersect(JVector origin, JVector direction)
        {
            return SegmentIntersect(ref origin, ref direction);
        }

        public bool RayIntersect(JVector origin, JVector direction)
        {
            return RayIntersect(ref origin, ref direction);
        }

        /// <summary>
        /// Checks wether a point is within a box or not.
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public ContainmentType Contains(JVector point)
        {
            return Contains(ref point);
        }

        /// <summary>
        /// Checks whether a point is inside, outside or intersecting
        /// a point.
        /// </summary>
        /// <param name="point">A point in space.</param>
        /// <returns>The ContainmentType of the point.</returns>
        public ContainmentType Contains(ref JVector point)
        {
            return Min.X <= point.X && point.X <= Max.X &&
                   Min.Y <= point.Y && point.Y <= Max.Y &&
                   Min.Z <= point.Z && point.Z <= Max.Z ? ContainmentType.Contains : ContainmentType.Disjoint;
        }

        /// <summary>
        /// Retrieves the 8 corners of the box.
        /// </summary>
        /// <returns>An array of 8 JVector entries.</returns>
        public void GetCorners(JVector[] corners)
        {
            corners[0] = new JVector(Min.X, Max.Y, Max.Z);
            corners[1] = new JVector(Max.X, Max.Y, Max.Z);
            corners[2] = new JVector(Max.X, Min.Y, Max.Z);
            corners[3] = new JVector(Min.X, Min.Y, Max.Z);
            corners[4] = new JVector(Min.X, Max.Y, Min.Z);
            corners[5] = new JVector(Max.X, Max.Y, Min.Z);
            corners[6] = new JVector(Max.X, Min.Y, Min.Z);
            corners[7] = new JVector(Min.X, Min.Y, Min.Z);
        }


        public void AddPoint(JVector point)
        {
            AddPoint(ref point);
        }

        public void AddPoint(ref JVector point)
        {
            Max = JVector.Max(Max, point);
            Min = JVector.Min(Min, point);
        }

        /// <summary>
        /// Expands a bounding box with the volume 0 by all points
        /// given.
        /// </summary>
        /// <param name="points">A array of JVector.</param>
        /// <returns>The resulting bounding box containing all points.</returns>
        public static JBBox CreateFromPoints(JVector[] points)
        {
            var vector3 = new JVector(float.MaxValue);
            var vector2 = new JVector(float.MinValue);

            for (var i = 0; i < points.Length; i++)
            {
                vector3 = JVector.Min(vector3, points[i]);
                vector2 = JVector.Max(vector2, points[i]);
            }
            return new JBBox(vector3, vector2);
        }

        /// <summary>
        /// Checks whether another bounding box is inside, outside or intersecting
        /// this box. 
        /// </summary>
        /// <param name="box">The other bounding box to check.</param>
        /// <returns>The ContainmentType of the box.</returns>
        public ContainmentType Contains(JBBox box)
        {
            return Contains(ref box);
        }

        /// <summary>
        /// Checks whether another bounding box is inside, outside or intersecting
        /// this box. 
        /// </summary>
        /// <param name="box">The other bounding box to check.</param>
        /// <returns>The ContainmentType of the box.</returns>
        public ContainmentType Contains(ref JBBox box)
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
            var vector2 = JVector.Min(original.Min, additional.Min);
            var vector = JVector.Max(original.Max, additional.Max);
            result.Min = vector2;
            result.Max = vector;
        }

        public JVector Center => (Min + Max)* (1.0f /2.0f);

        internal float Perimeter =>
            2.0f * ((Max.X - Min.X) * (Max.Y - Min.Y) +
                    (Max.X - Min.X) * (Max.Z - Min.Z) +
                    (Max.Z - Min.Z) * (Max.Y - Min.Y));
    }
}
