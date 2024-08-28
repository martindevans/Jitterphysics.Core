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
using Jitter.LinearMath;

namespace Jitter.Collision.Shapes
{
    /// <summary>
    /// A <see cref="BaseShape"/> representing a sphere.
    /// </summary>
    public class SphereShape : BaseShape
    {
        private float _radius;

        /// <summary>
        /// The radius of the sphere.
        /// </summary>
        public float Radius
        {
            get => _radius;
            set { _radius = value; UpdateShape(); }
        }

        /// <summary>
        /// Creates a new instance of the SphereShape class.
        /// </summary>
        /// <param name="radius">The radius of the sphere</param>
        public SphereShape(float radius)
        {
            Radius = radius;
        }

        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        public override Vector3 SupportMapping(Vector3 direction)
        {
            var result = direction;
            result = Vector3.Normalize(result);

            result *= Radius;
            return result;
        }

        /// <summary>
        /// Calculates the bounding box of the sphere.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <returns>The resulting axis aligned bounding box.</returns>
        public override JBBox GetBoundingBox(JMatrix orientation)
        {
            JBBox box;
            box.Min.X = -Radius;
            box.Min.Y = -Radius;
            box.Min.Z = -Radius;
            box.Max.X = Radius;
            box.Max.Y = Radius;
            box.Max.Z = Radius;
            return box;
        }

        /// <inheritdoc />
        public override void CalculateMassInertia()
        {
            Mass = 4.0f / 3.0f * MathF.PI * Radius * Radius * Radius;

            // (0,0,0) is the center of mass, so only
            // the main matrix elements are != 0
            var i = JMatrix.Identity;
            i.M11 = 0.4f * Mass * Radius * Radius;
            i.M22 = 0.4f * Mass * Radius * Radius;
            i.M33 = 0.4f * Mass * Radius * Radius;
            Inertia = i;
        }
    }
}
