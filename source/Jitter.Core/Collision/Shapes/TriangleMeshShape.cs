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

using System.Collections.Generic;
using System.Numerics;
using Jitter.LinearMath;

namespace Jitter.Collision.Shapes
{

    /// <summary>
    /// A <see cref="BaseShape"/> representing a triangleMesh.
    /// </summary>
    public class TriangleMeshShape : Multishape
    {
        private readonly List<int> potentialTriangles = new();
        private readonly Octree octree;

        /// <summary>
        /// Expands the triangles by the specified amount.
        /// This stabilizes collision detection for flat shapes.
        /// </summary>
        public float SphericalExpansion { get; set; } = 0.05f;

        public bool FlipNormals { get; set; }

        /// <summary>
        /// Creates a new instance if the TriangleMeshShape class.
        /// </summary>
        /// <param name="octree">The octree which holds the triangles
        /// of a mesh.</param>
        public TriangleMeshShape(Octree octree)
        {
            this.octree = octree;
            UpdateShape();
        }

        internal TriangleMeshShape() { }

 
        protected override Multishape CreateWorkingClone()
        {
            var clone = new TriangleMeshShape(octree)
            {
                SphericalExpansion = SphericalExpansion,
            };
            return clone;
        }


        /// <summary>
        /// Passes a axis aligned bounding box to the shape where collision
        /// could occour.
        /// </summary>
        /// <param name="box">The bounding box where collision could occur.</param>
        /// <returns>The upper index with which <see cref="SetCurrentShape"/> can be 
        /// called.</returns>
        public override int Prepare(ref JBBox box)
        {
            potentialTriangles.Clear();

            var exp = box;

            exp.Min.X -= SphericalExpansion;
            exp.Min.Y -= SphericalExpansion;
            exp.Min.Z -= SphericalExpansion;
            exp.Max.X += SphericalExpansion;
            exp.Max.Y += SphericalExpansion;
            exp.Max.Z += SphericalExpansion;

            octree.GetTrianglesIntersectingtAABox(potentialTriangles, ref exp);

            return potentialTriangles.Count;
        }

        public override void MakeHull(List<Vector3> triangleList, int generationThreshold)
        {
            var large = JBBox.LargeBox;

            var indices = new List<int>();
            octree.GetTrianglesIntersectingtAABox(indices, ref large);

            for (var i = 0; i < indices.Count; i++)
            {
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I0));
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I1));
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I2));
            }

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayDelta"></param>
        /// <returns></returns>
        public override int Prepare(ref Vector3 rayOrigin, ref Vector3 rayDelta)
        {
            potentialTriangles.Clear();

            var expDelta = Vector3.Normalize(rayDelta);
            expDelta = rayDelta + expDelta * SphericalExpansion;

            octree.GetTrianglesIntersectingRay(potentialTriangles, rayOrigin, expDelta);

            return potentialTriangles.Count;
        }

        private readonly Vector3[] vecs = new Vector3[3];

        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        public override Vector3 SupportMapping(Vector3 direction)
        {
            var exp = Vector3.Normalize(direction);
            exp *= SphericalExpansion;

            var min = Vector3.Dot(vecs[0], direction);
            var minIndex = 0;
            var dot = Vector3.Dot(vecs[1], direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 1;
            }
            dot = Vector3.Dot(vecs[2], direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 2;
            }

            var result = vecs[minIndex] + exp;
            return result;
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape. This includes
        /// the whole shape.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <returns>The axis aligned bounding box of the shape.</returns>
        public override JBBox GetBoundingBox(JMatrix orientation)
        {
            var box = octree.rootNodeBox;

            box.Min.X -= SphericalExpansion;
            box.Min.Y -= SphericalExpansion;
            box.Min.Z -= SphericalExpansion;
            box.Max.X += SphericalExpansion;
            box.Max.Y += SphericalExpansion;
            box.Max.Z += SphericalExpansion;

            box.Transform(ref orientation);
            return box;
        }

        /// <summary>
        /// Sets the current shape. First <see cref="Prepare"/> has to be called.
        /// After SetCurrentShape the shape immitates another shape.
        /// </summary>
        /// <param name="index"></param>
        public override void SetCurrentShape(int index)
        {
            vecs[0] = octree.GetVertex(octree.tris[potentialTriangles[index]].I0);
            vecs[1] = octree.GetVertex(octree.tris[potentialTriangles[index]].I1);
            vecs[2] = octree.GetVertex(octree.tris[potentialTriangles[index]].I2);

            var sum = vecs[0];
            var value2 = vecs[1];
            sum += value2;
            var value3 = vecs[2];
            sum += value3;
            sum *= (1.0f / 3.0f);

      
            geomCen = sum;

            var value4 = vecs[0];
            sum = vecs[1] - value4;
            var value5 = vecs[0];
            normal = vecs[2] - value5;
            normal = Vector3.Cross(sum, normal);

            if (FlipNormals) normal = -normal;
        }

        private Vector3 normal = JVectorExtensions.Up;

        public void CollisionNormal(out Vector3 normal)
        {
            normal = this.normal;
        }
    }

}
