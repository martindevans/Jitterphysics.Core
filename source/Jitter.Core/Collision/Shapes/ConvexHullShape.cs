﻿/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
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

using System.Collections.Generic;
using System.Numerics;

namespace Jitter.Collision.Shapes
{

    /// <summary>
    /// ConvexHullShape class.
    /// </summary>
    public class ConvexHullShape : BaseShape
    {
        private List<Vector3> vertices;

        private Vector3 shifted;

        /// <summary>
        /// Constructor of ConvexHullShape class.
        /// </summary>
        /// <param name="vertices">A list containing all vertices defining
        /// the convex hull.</param>
        public ConvexHullShape(List<Vector3> vertices)
        {
            this.vertices = vertices;
            UpdateShape();
        }

        public Vector3 Shift => -1 * shifted;

        public override void CalculateMassInertia()
        {
            mass = BaseShape.CalculateMassInertia(this, out shifted, out inertia);
        }

        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public override Vector3 SupportMapping(Vector3 direction)
        {
            var maxDotProduct = float.MinValue;
            var maxIndex = 0;
            float dotProduct;

            for (var i = 0; i < vertices.Count; i++)
            {
                dotProduct = Vector3.Dot(vertices[i], direction);
                if (dotProduct > maxDotProduct)
                {
                    maxDotProduct = dotProduct;
                    maxIndex = i;
                }
            }

            var result = vertices[maxIndex] - shifted;
            return result;
        }
    }
}
