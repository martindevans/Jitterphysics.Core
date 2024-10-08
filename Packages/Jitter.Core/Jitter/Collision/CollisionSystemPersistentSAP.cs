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

#nullable enable

using System;
using System.Collections.Generic;
using System.Numerics;
using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;

namespace Jitter.Collision
{
    /// <summary>
    /// Full 3-Axis SweepAndPrune using persistent updates.
    /// </summary>
    public class CollisionSystemPersistentSAP
        : CollisionSystem
    {
        private const int AddedObjectsBruteForceIsUsed = 250;

        private class SweepPoint
            : IComparable<SweepPoint>
        {
            public readonly RigidBody Body;
            public readonly bool Begin;
            public readonly int Axis;

            public SweepPoint(RigidBody body, bool begin, int axis)
            {
                Body = body;
                Begin = begin;
                Axis = axis;
            }

            public float Value
            {
                get
                {
                    if (Begin)
                        return Body.BoundingBox.Min.Element(Axis);
                    else
                        return Body.BoundingBox.Max.Element(Axis);
                }
            }

            public int CompareTo(SweepPoint other)
            {
                return Value.CompareTo(other.Value);
            }
        }

        private readonly struct OverlapPair
            : IEquatable<OverlapPair>
        {
            public readonly RigidBody Entity1;
            public readonly RigidBody Entity2;

            /// <summary>
            /// Initializes a new instance of the BodyPair class.
            /// </summary>
            /// <param name="entity1"></param>
            /// <param name="entity2"></param>
            public OverlapPair(RigidBody entity1, RigidBody entity2)
            {
                Entity1 = entity1;
                Entity2 = entity2;
            }

            public bool Equals(OverlapPair other)
            {
                return other.Entity1.Equals(Entity1) && other.Entity2.Equals(Entity2) ||
                       other.Entity1.Equals(Entity2) && other.Entity2.Equals(Entity1);
            }

            /// <summary>
            /// Checks if two objects are equal.
            /// </summary>
            /// <param name="obj">The object to check against.</param>
            /// <returns>Returns true if they are equal, otherwise false.</returns>
            public override bool Equals(object? obj)
            {
                if (obj is not OverlapPair pair)
                    return false;

                return Equals(pair);
            }

            /// <summary>
            /// Returns the hashcode of the BodyPair.
            /// The hashcode is the same if an BodyPair contains the same bodies.
            /// </summary>
            /// <returns></returns>
            public override int GetHashCode()
            {
                // Intentional order invariant hash
                return unchecked(Entity1.GetHashCode() + Entity2.GetHashCode());
            }
        }

        // not needed anymore
        private readonly List<RigidBody> bodyList = new();

        private readonly List<SweepPoint> axis1 = new();
        private readonly List<SweepPoint> axis2 = new();
        private readonly List<SweepPoint> axis3 = new();

        private readonly HashSet<OverlapPair> fullOverlaps = new();

        private readonly List<RigidBody> activeList = new();

        private void DirtySortAxis(List<SweepPoint> axis)
        {
            axis.Sort();
            activeList.Clear();

            for (var i = 0; i < axis.Count; i++)
            {
                var keyelement = axis[i];

                if (keyelement.Begin)
                {
                    foreach (var body in activeList)
                        if (CheckBoundingBoxes(body,keyelement.Body)) 
                            fullOverlaps.Add(new(body, keyelement.Body));

                    activeList.Add(keyelement.Body);
                }
                else
                {
                    activeList.Remove(keyelement.Body);
                }
            }
        }

        private void SortAxis(List<SweepPoint> axis)
        {
            for (var j = 1; j < axis.Count; j++)
            {
                var keyelement = axis[j];
                var key = keyelement.Value;

                var i = j - 1;

                while (i >= 0 && axis[i].Value > key)
                {
                    var swapper = axis[i];

                    if (keyelement.Begin && !swapper.Begin)
                    {
                        if (CheckBoundingBoxes(swapper.Body, keyelement.Body))
                            fullOverlaps.Add(new(swapper.Body, keyelement.Body));
                    }

                    if (!keyelement.Begin && swapper.Begin)
                        fullOverlaps.Remove(new(swapper.Body, keyelement.Body));

                    axis[i + 1] = swapper;
                    i -= 1;
                }
                axis[i + 1] = keyelement;
            }
        }

        private int addCounter;
        public override void AddEntity(RigidBody body)
        {
            bodyList.Add(body);

            axis1.Add(new(body, true, 0));
            axis1.Add(new(body, false, 0));

            axis2.Add(new(body, true, 1));
            axis2.Add(new(body, false, 1));

            axis3.Add(new(body, true, 2));
            axis3.Add(new(body, false, 2));

            addCounter++;
        }

        private readonly Stack<OverlapPair> deprecated = new();

        public override bool RemoveEntity(RigidBody body)
        {
            var count = 0;
            for (var i = 0; i < axis1.Count; i++)
            {
                if (axis1[i].Body == body)
                {
                    count++;
                    axis1.RemoveAt(i);
                    if (count == 2)
                        break;
                    i--;
                }
            }

            count = 0;
            for (var i = 0; i < axis2.Count; i++)
            {
                if (axis2[i].Body == body)
                {
                    count++;
                    axis2.RemoveAt(i);
                    if (count == 2)
                        break;
                    i--;
                }
            }

            count = 0;
            for (var i = 0; i < axis3.Count; i++)
            {
                if (axis3[i].Body == body)
                {
                    count++;
                    axis3.RemoveAt(i);
                    if (count == 2)
                        break;
                    i--;
                }
            }

            foreach (var pair in fullOverlaps)
                if (pair.Entity1 == body || pair.Entity2 == body)
                    deprecated.Push(pair);
            while (deprecated.Count > 0)
                fullOverlaps.Remove(deprecated.Pop());

            bodyList.Remove(body);

            return true;
        }

        private bool swapOrder;

        /// <summary>
        /// Tells the collisionsystem to check all bodies for collisions. Hook into the
        /// <see cref="CollisionSystem.PassedBroadphase"/>
        /// and <see cref="CollisionSystem.CollisionDetected"/> events to get the results.
        /// </summary>
        public override void Detect()
        {
            if (addCounter > AddedObjectsBruteForceIsUsed)
            {
                fullOverlaps.Clear();

                DirtySortAxis(axis1);
                DirtySortAxis(axis2);
                DirtySortAxis(axis3);
            }
            else
            {
                SortAxis(axis1);
                SortAxis(axis2);
                SortAxis(axis3);
            }

            addCounter = 0;

            foreach (var key in fullOverlaps)
            {
                if (CheckBothStaticOrInactive(key.Entity1, key.Entity2)) continue;

                if (RaisePassedBroadphase(key.Entity1, key.Entity2))
                {
                    {
                        if (swapOrder) { Detect(key.Entity1, key.Entity2); }
                        else Detect(key.Entity2, key.Entity1);
                    }

                    swapOrder = !swapOrder;
                }
            }
        }

        // okay, people often say raycasting can be made faster using the sweep
        // and prune data. (sorted axis lists). That's only partly correct,
        // the method commented out below uses the non-uniform voxelgrid
        // approach (best approach known so far). 
        // but this is O(n) for long rays too and it even takes
        // more time. Just for short rays it has a small benefit.
        // it also gives the hits in order but the startposition problem
        // is unsolved - so it starts from outside the broadphase.

        //public void QueryRay(HashSet<IBroadphaseEntity> entities,Vector3 rayOrigin, Vector3 rayDirection)
        //{
        //    rayDirection.Normalize();

        //    int index1 = 0,index2 = 0,index3 = 0;

        //    if (rayDirection.X < 0.0f) index1 = axis1.Count - 1;
        //    if (rayDirection.Y < 0.0f) index2 = axis2.Count - 1;
        //    if (rayDirection.Z < 0.0f) index3 = axis3.Count - 1;

        //    int steps = 0;

        //    while (true)
        //    {
        //        steps++;
        //        float distance1 = (axis1[index1].Value - rayOrigin.X) / rayDirection.X;
        //        float distance2 = (axis2[index2].Value - rayOrigin.Y) / rayDirection.Y;
        //        float distance3 = (axis3[index3].Value - rayOrigin.Z) / rayDirection.Z;


        //        float dist1 = Math.Abs(distance1);
        //        float dist2 = Math.Abs(distance2);
        //        float dist3 = Math.Abs(distance3);

        //        if (dist1 < dist2)
        //        {
        //            if (dist3 < dist1)
        //            {

        //                if (axis3[index3].Begin)
        //                {
        //                    if (axis3[index3].Body.BoundingBox.RayIntersect(rayOrigin, rayDirection)) entities.Add(axis3[index3].Body);
        //                }

        //                rayOrigin = rayOrigin + distance3 * rayDirection;

        //                index3 += (rayDirection.Z > 0.0f) ? 1 : -1;
        //                if (index3 >= axis3.Count || index3 < 0) break;
                       
        //            }
        //            else
        //            {

        //                if (axis1[index1].Begin)
        //                {
        //                    if (axis1[index1].Body.BoundingBox.RayIntersect(rayOrigin, rayDirection)) entities.Add(axis1[index1].Body);
        //                }

        //                rayOrigin = rayOrigin + distance1 * rayDirection;

        //                index1 += (rayDirection.X > 0.0f) ? 1 : -1;
        //                if (index1 >= axis1.Count || index1 < 0) break;
        //            }
        //        }
        //        else
        //        {
        //            if (dist2 < dist3)
        //            {
        //                if (axis2[index2].Begin)
        //                {
        //                    if (axis2[index2].Body.BoundingBox.RayIntersect(rayOrigin, rayDirection)) entities.Add(axis2[index2].Body);
        //                }

        //                rayOrigin = rayOrigin + distance2 * rayDirection;

        //                index2 += (rayDirection.Y > 0.0f) ? 1 : -1;
        //                if (index2 >= axis2.Count || index2 < 0) break;

        //            }
        //            else
        //            {

        //                if (axis3[index3].Begin)
        //                {
        //                    if (axis3[index3].Body.BoundingBox.RayIntersect(rayOrigin, rayDirection)) entities.Add(axis3[index3].Body);
        //                }

        //                rayOrigin = rayOrigin + distance3 * rayDirection;

        //                index3 += (rayDirection.Z > 0.0f) ? 1 : -1;
        //                if (index3 >= axis3.Count || index3 < 0) break;
        //            }
        //        }
        //    }

        //    System.Diagnostics.Debug.WriteLine(steps);
        //}


        /// <summary>
        /// Sends a ray (definied by start and direction) through the scene (all bodies added).
        /// NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        public override bool Raycast(Vector3 rayOrigin, Vector3 rayDirection, RaycastCallback raycast, out RigidBody body, out Vector3 normal, out float fraction)
        {
            body = null!;
            normal = default;
            fraction = float.MaxValue;

            var result = false;

            // TODO: This can be done better in CollisionSystemPersistenSAP
            foreach (var e in bodyList)
            {
                {
                    if (Raycast(e, rayOrigin, rayDirection, out var tempNormal, out var tempFraction))
                    {
                        if (tempFraction < fraction && (raycast == null || raycast(e, tempNormal, tempFraction)))
                        {
                            body = e;
                            normal = tempNormal;
                            fraction = tempFraction;
                            result = true;
                        }
                    }
                }
            }

            return result;
        }


        /// <summary>
        /// Raycasts a single body. NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        public override bool Raycast(RigidBody body, Vector3 rayOrigin, Vector3 rayDirection, out Vector3 normal, out float fraction)
        {
            fraction = float.MaxValue;
            normal = default;

            if (!body.BoundingBox.RayIntersect(rayOrigin, rayDirection)) return false;

            if (body.Shape is Multishape ms)
            {
                ms = ms.RequestWorkingClone();
                try
                {
                    var multiShapeCollides = false;

                    var transformedOrigin = rayOrigin - body.position;
                    transformedOrigin = transformedOrigin.Transform(body.invOrientation);
                    var transformedDirection = rayDirection.Transform(body.invOrientation);

                    var msLength = ms.Prepare(ref transformedOrigin, ref transformedDirection);

                    for (var i = 0; i < msLength; i++)
                    {
                        ms.SetCurrentShape(i);

                        if (GJKCollide.Raycast(ms, ref body.orientation, ref body.position,
                                ref rayOrigin, ref rayDirection, out var tempFraction, out var tempNormal))
                        {
                            if (tempFraction < fraction)
                            {
                                if (UseTriangleMeshNormal && ms is TriangleMeshShape shape)
                                {
                                    shape.CollisionNormal(out tempNormal);
                                    tempNormal = tempNormal.Transform(body.orientation);
                                    tempNormal = -tempNormal;
                                }

                                normal = tempNormal;
                                fraction = tempFraction;
                                multiShapeCollides = true;
                            }
                        }
                    }

                    return multiShapeCollides;
                }
                finally
                {
                    ms.ReturnWorkingClone();
                }
            }
            else
            {
                return GJKCollide.Raycast(body.Shape, ref body.orientation, ref body.position,
                    ref rayOrigin, ref rayDirection, out fraction, out normal);
            }


        }
    }
}
