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
using System.Collections.Generic;
using Jitter.LinearMath;

namespace Jitter.Collision
{

    /// <summary>
    /// structure used to set up the mesh
    /// </summary>
    public struct TriangleVertexIndices
    {
        /// <summary>
        /// The first index.
        /// </summary>
        public int I0;
        /// <summary>
        /// The second index.
        /// </summary>
        public int I1;
        /// <summary>
        /// The third index.
        /// </summary>
        public int I2;

        /// <summary>
        /// Initializes a new instance of the TriangleVertexIndex structure.
        /// </summary>
        /// <param name="i0">The index of the first vertex.</param>
        /// <param name="i1">The index of the second vertex.</param>
        /// <param name="i2">The index of the third vertex.</param>
        public TriangleVertexIndices(int i0, int i1, int i2)
        {
            I0 = i0;
            I1 = i1;
            I2 = i2;
        }

        /// <summary>
        /// Sets the values for the indices.
        /// </summary>
        /// <param name="i0">The index of the first vertex.</param>
        /// <param name="i1">The index of the second vertex.</param>
        /// <param name="i2">The index of the third vertex.</param>
        public void Set(int i0, int i1, int i2)
        {
            I0 = i0; I1 = i1; I2 = i2;
        }
    }


    /// <summary>
    /// An octree implementation.
    /// </summary>
    public class Octree
    {
        /// <summary>
        /// endices into the children - P means "plus" and M means "minus" and the
        /// letters are xyz. So PPM means +ve x, +ve y, -ve z
        /// </summary>
        [Flags]
        private enum EChild
        {
            XP = 0x1,
            YP = 0x2,
            ZP = 0x4,
            PPP = XP | YP | ZP,
            PPM = XP | YP,
            PMP = XP | ZP,
            PMM = XP,
            MPP = YP | ZP,
            MPM = YP,
            MMP = ZP,
            MMM = 0x0,
        }

        private struct Node
        {
            public UInt16[] nodeIndices;
            public int[] triIndices;
            public JBBox box;
        }

        private class BuildNode
        {
            public int childType; // will default to MMM (usually ECHild but can also be -1)
            public List<int> nodeIndices = new List<int>();
            public List<int> triIndices = new List<int>();
            public JBBox box;
        }

        private JVector[] positions;
        private JBBox[] triBoxes;
        private Node[] nodes;
        //private UInt16[] nodeStack;
        internal TriangleVertexIndices[] tris;
        internal JBBox rootNodeBox;

        /// <summary>
        /// Gets the root node box containing the whole octree.
        /// </summary>
        public JBBox RootNodeBox => rootNodeBox;

        /// <summary>
        /// Clears the octree.
        /// </summary>
        public void Clear()
        {
            positions = null;
            triBoxes = null;
            tris = null;
            nodes = null;
            nodeStackPool.ResetResourcePool();
        }

        /// <summary>
        /// Sets new triangles.
        /// </summary>
        /// <param name="positions">Vertices.</param>
        /// <param name="tris">Indices.</param>
        public void SetTriangles(List<JVector> positions, List<TriangleVertexIndices> tris)
        {
            // copy the position data into a array
            this.positions = new JVector[positions.Count];
            positions.CopyTo(this.positions);

            // copy the triangles
            this.tris = new TriangleVertexIndices[tris.Count];
            tris.CopyTo(this.tris);
        }

        /// <summary>
        /// Builds the octree.
        /// </summary>
        public void BuildOctree()
        {
            // create tri and tri bounding box arrays
            triBoxes = new JBBox[tris.Length];

            // create an infinite size root box
            rootNodeBox = new JBBox(new JVector(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity),
                                           new JVector(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity));


            for (var i = 0; i < tris.Length; i++)
            {
                triBoxes[i].Min = JVector.Min(positions[tris[i].I1], positions[tris[i].I2]);
                triBoxes[i].Min = JVector.Min(positions[tris[i].I0], triBoxes[i].Min);

                triBoxes[i].Max = JVector.Max(positions[tris[i].I1], positions[tris[i].I2]);
                triBoxes[i].Max = JVector.Max(positions[tris[i].I0], triBoxes[i].Max);

                // get size of the root box
                rootNodeBox.Min = JVector.Min(rootNodeBox.Min, triBoxes[i].Min);
                rootNodeBox.Max = JVector.Max(rootNodeBox.Max, triBoxes[i].Max);
            }

            var buildNodes = new List<BuildNode>();
            buildNodes.Add(new BuildNode());
            buildNodes[0].box = rootNodeBox;

            var children = new JBBox[8];
            for (var triNum = 0; triNum < tris.Length; triNum++)
            {
                var nodeIndex = 0;
                var box = rootNodeBox;

                while (box.Contains(ref triBoxes[triNum]) == JBBox.ContainmentType.Contains)
                {
                    var childCon = -1;
                    for (var i = 0; i < 8; ++i)
                    {
                        CreateAABox(ref box, (EChild)i,out children[i]);
                        if (children[i].Contains(ref triBoxes[triNum]) == JBBox.ContainmentType.Contains)
                        {
                            // this box contains the tri, it can be the only one that does,
                            // so we can stop our child search now and recurse into it
                            childCon = i;
                            break;
                        }
                    }

                    // no child contains this tri completely, so it belong in this node
                    if (childCon == -1)
                    {
                        buildNodes[nodeIndex].triIndices.Add(triNum);
                        break;
                    }
                    else
                    {
                        // do we already have this child
                        var childIndex = -1;
                        for (var index = 0; index < buildNodes[nodeIndex].nodeIndices.Count; ++index)
                        {
                            if (buildNodes[buildNodes[nodeIndex].nodeIndices[index]].childType == childCon)
                            {
                                childIndex = index;
                                break;
                            }
                        }
                        if (childIndex == -1)
                        {
                            // nope create child
                            var parentNode = buildNodes[nodeIndex];
                            var newNode = new BuildNode();
                            newNode.childType = childCon;
                            newNode.box = children[childCon];
                            buildNodes.Add(newNode);

                            nodeIndex = buildNodes.Count - 1;
                            box = children[childCon];
                            parentNode.nodeIndices.Add(nodeIndex);
                        }
                        else
                        {
                            nodeIndex = buildNodes[nodeIndex].nodeIndices[childIndex];
                            box = children[childCon];
                        }
                    }
                }
            }

            // now convert to the tighter Node from BuildNodes
            nodes = new Node[buildNodes.Count];
            nodeStackPool = new ArrayResourcePool<ushort>(buildNodes.Count);
            //nodeStack = new UInt16[buildNodes.Count];
            for (var i = 0; i < nodes.Length; i++)
            {
                nodes[i].nodeIndices = new UInt16[buildNodes[i].nodeIndices.Count];
                for (var index = 0; index < nodes[i].nodeIndices.Length; ++index)
                {
                    nodes[i].nodeIndices[index] = (UInt16)buildNodes[i].nodeIndices[index];
                }

                nodes[i].triIndices = new int[buildNodes[i].triIndices.Count];
                buildNodes[i].triIndices.CopyTo(nodes[i].triIndices);
                nodes[i].box = buildNodes[i].box;
            }
            buildNodes.Clear(); buildNodes = null;
        }

        /// <summary>
        /// Initializes a new instance of the Octree class.
        /// </summary>
        /// <param name="positions">Vertices.</param>
        /// <param name="tris">Indices.</param>
        public Octree(List<JVector> positions, List<TriangleVertexIndices> tris)
        {
            SetTriangles(positions, tris);
            BuildOctree();
        }

        /// <summary>
        /// Create a bounding box appropriate for a child, based on a parents AABox
        /// </summary>
        /// <param name="aabb"></param>
        /// <param name="child"></param>
        /// <param name="result"></param>
        private void CreateAABox(ref JBBox aabb, EChild child,out JBBox result)
        {
            var dims = JVector.Subtract(aabb.Max, aabb.Min);
            dims = JVector.Multiply(dims, 0.5f);

            var offset = JVector.Zero;

            switch (child)
            {
                case EChild.PPP: offset = new JVector(1, 1, 1); break;
                case EChild.PPM: offset = new JVector(1, 1, 0); break;
                case EChild.PMP: offset = new JVector(1, 0, 1); break;
                case EChild.PMM: offset = new JVector(1, 0, 0); break;
                case EChild.MPP: offset = new JVector(0, 1, 1); break;
                case EChild.MPM: offset = new JVector(0, 1, 0); break;
                case EChild.MMP: offset = new JVector(0, 0, 1); break;
                case EChild.MMM: offset = new JVector(0, 0, 0); break;

                default:
                    System.Diagnostics.Debug.WriteLine("Octree.CreateAABox  got impossible child");
                    break;
            }

            result = new JBBox();
            result.Min = new JVector(offset.X * dims.X, offset.Y * dims.Y, offset.Z * dims.Z);
            result.Min = JVector.Add(result.Min, aabb.Min);

            result.Max = JVector.Add(result.Min, dims);

            // expand it just a tiny bit just to be safe!
            var extra = 0.00001f;

            var temp = JVector.Multiply(dims, extra);
            result.Min = JVector.Subtract(result.Min, temp);
            result.Max = JVector.Add(result.Max, temp);
        }

        private void GatherTriangles(int nodeIndex, ref List<int> tris)
        {
            // add this nodes triangles
            tris.AddRange(nodes[nodeIndex].triIndices);

            // recurse into this nodes children
            var numChildren = nodes[nodeIndex].nodeIndices.Length;
            for (var i = 0; i < numChildren; ++i)
            {
                int childNodeIndex = nodes[nodeIndex].nodeIndices[i];
                GatherTriangles(childNodeIndex, ref tris);
            }
        }


        /// <summary>
        /// Returns all triangles which intersect the given axis aligned bounding box.
        /// </summary>
        /// <param name="triangles">The list to add the triangles to.</param>
        /// <param name="testBox">The axis alignes bounding box.</param>
        /// <returns></returns>
        public int GetTrianglesIntersectingtAABox(List<int> triangles, ref JBBox testBox)
        {
            if (nodes.Length == 0)
                return 0;
            var curStackIndex = 0;
            var endStackIndex = 1;

            var nodeStack = nodeStackPool.GetNew();

            nodeStack[0] = 0;

            var triCount = 0;

            while (curStackIndex < endStackIndex)
            {
                var nodeIndex = nodeStack[curStackIndex];
                curStackIndex++;
                if (nodes[nodeIndex].box.Contains(ref testBox) != JBBox.ContainmentType.Disjoint)
                {
                    for (var i = 0; i < nodes[nodeIndex].triIndices.Length; ++i)
                    {
                        if (triBoxes[nodes[nodeIndex].triIndices[i]].Contains(ref testBox) != JBBox.ContainmentType.Disjoint)
                        {
                            triangles.Add(nodes[nodeIndex].triIndices[i]);
                            triCount++;
                        }
                    }

                    var numChildren = nodes[nodeIndex].nodeIndices.Length;
                    for (var i = 0; i < numChildren; ++i)
                    {
                        nodeStack[endStackIndex++] = nodes[nodeIndex].nodeIndices[i];
                    }
                }
            }

            nodeStackPool.GiveBack(nodeStack);

            return triCount;
        }

        private ArrayResourcePool<UInt16> nodeStackPool;

        /// <summary>
        /// Returns all triangles which intersect the given axis aligned bounding box.
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayDelta"></param>
        /// <param name="triangles"></param>
        /// <returns></returns>
        public int GetTrianglesIntersectingRay(List<int> triangles, JVector rayOrigin, JVector rayDelta)
        {
            if (nodes.Length == 0)
                return 0;
            var curStackIndex = 0;
            var endStackIndex = 1;

            var nodeStack = nodeStackPool.GetNew();
            nodeStack[0] = 0;

            var triCount = 0;

            while (curStackIndex < endStackIndex)
            {
                var nodeIndex = nodeStack[curStackIndex];
                curStackIndex++;
                if (nodes[nodeIndex].box.SegmentIntersect(ref rayOrigin, ref rayDelta))
                {
                    for (var i = 0; i < nodes[nodeIndex].triIndices.Length; ++i)
                    {
                        if (triBoxes[nodes[nodeIndex].triIndices[i]].SegmentIntersect(ref rayOrigin, ref rayDelta))
                        {
                            triangles.Add(nodes[nodeIndex].triIndices[i]);
                            triCount++;
                        }
                    }

                    var numChildren = nodes[nodeIndex].nodeIndices.Length;
                    for (var i = 0; i < numChildren; ++i)
                    {
                        nodeStack[endStackIndex++] = nodes[nodeIndex].nodeIndices[i];
                    }
                }
            }

            nodeStackPool.GiveBack(nodeStack);
            return triCount;
        }

        /// <summary>
        /// Gets the indices of a triangle by index.
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns>The indices of a triangle.</returns>
        public TriangleVertexIndices GetTriangleVertexIndex(int index)
        {
            return tris[index];
        }

        /// <summary>
        /// Gets a vertex from the vertex list.
        /// </summary>
        /// <param name="vertex">The index of the vertex</param>
        /// <returns></returns>
        public JVector GetVertex(int vertex)
        {
            return positions[vertex];
        }

        /// <summary>
        /// Gets a vertex from the vertex list.
        /// </summary>
        /// <param name="vertex">The index of the vertex</param>
        /// <param name="result"></param>
        public void GetVertex(int vertex, out JVector result)
        {
            result = positions[vertex];
        }

        /// <summary>
        /// Gets the number of triangles within this octree.
        /// </summary>
        public int NumTriangles
        {
            get { return tris.Length; }
        }
    }
}
