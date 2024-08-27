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
using System.Numerics;
using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;

namespace Jitter.Collision
{
    /// <summary>
    /// Full 3-Axis SweepAndPrune using persistent updates.
    /// </summary>
    public class CollisionSystemPersistentSAP : CollisionSystem
    {
        private const int AddedObjectsBruteForceIsUsed = 250;

        private class SweepPoint
        {
            public IBroadphaseEntity Body;
            public bool Begin;
            public int Axis;

            public SweepPoint(IBroadphaseEntity body, bool begin, int axis)
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
                    {
                        if (Axis == 0) return Body.BoundingBox.Min.X;
                        else if (Axis == 1) return Body.BoundingBox.Min.Y;
                        else return Body.BoundingBox.Min.Z;
                    }
                    else
                    {
                        if (Axis == 0) return Body.BoundingBox.Max.X;
                        else if (Axis == 1) return Body.BoundingBox.Max.Y;
                        else return Body.BoundingBox.Max.Z;
                    }
                }
            }


        }

        private struct OverlapPair
        {
            // internal values for faster access within the engine
            public IBroadphaseEntity Entity1, Entity2;

            /// <summary>
            /// Initializes a new instance of the BodyPair class.
            /// </summary>
            /// <param name="entity1"></param>
            /// <param name="entity2"></param>
            public OverlapPair(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
            {
                Entity1 = entity1;
                Entity2 = entity2;
            }

            /// <summary>
            /// Don't call this, while the key is used in the arbitermap.
            /// It changes the hashcode of this object.
            /// </summary>
            /// <param name="entity1">The first body.</param>
            /// <param name="entity2">The second body.</param>
            internal void SetBodies(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
            {
                Entity1 = entity1;
                Entity2 = entity2;
            }

            /// <summary>
            /// Checks if two objects are equal.
            /// </summary>
            /// <param name="obj">The object to check against.</param>
            /// <returns>Returns true if they are equal, otherwise false.</returns>
            public override bool Equals(object obj)
            {
                var other = (OverlapPair)obj;
                return other.Entity1.Equals(Entity1) && other.Entity2.Equals(Entity2) ||
                       other.Entity1.Equals(Entity2) && other.Entity2.Equals(Entity1);
            }

            /// <summary>
            /// Returns the hashcode of the BodyPair.
            /// The hashcode is the same if an BodyPair contains the same bodies.
            /// </summary>
            /// <returns></returns>
            public override int GetHashCode()
            {
                return Entity1.GetHashCode() + Entity2.GetHashCode();
            }
        }

        // not needed anymore
        private List<IBroadphaseEntity> bodyList = new List<IBroadphaseEntity>();

        private List<SweepPoint> axis1 = new List<SweepPoint>();
        private List<SweepPoint> axis2 = new List<SweepPoint>();
        private List<SweepPoint> axis3 = new List<SweepPoint>();

        private HashSet<OverlapPair> fullOverlaps = new HashSet<OverlapPair>();

        Action<object> sortCallback;

        public CollisionSystemPersistentSAP()
        {
            sortCallback = SortCallback;
        }

        private int QuickSort(SweepPoint sweepPoint1, SweepPoint sweepPoint2)
        {
            var val1 = sweepPoint1.Value;
            var val2 = sweepPoint2.Value;

            if (val1 > val2) return 1;
            else if (val2 > val1) return -1;
            else return 0;
        }

        List<IBroadphaseEntity> activeList = new List<IBroadphaseEntity>();

        private void DirtySortAxis(List<SweepPoint> axis)
        {
            axis.Sort(QuickSort);
            activeList.Clear();

            for (var i = 0; i < axis.Count; i++)
            {
                var keyelement = axis[i];

                if (keyelement.Begin)
                {
                    foreach (var body in activeList)
                    {
                        if (CheckBoundingBoxes(body,keyelement.Body)) 
                            fullOverlaps.Add(new OverlapPair(body, keyelement.Body));
                    }

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
                        {
                            lock (fullOverlaps) fullOverlaps.Add(new OverlapPair(swapper.Body, keyelement.Body));
                        }
                    }

                    if (!keyelement.Begin && swapper.Begin)
                    {
                        lock (fullOverlaps) fullOverlaps.Remove(new OverlapPair(swapper.Body, keyelement.Body));
                    }

                    axis[i + 1] = swapper;
                    i -= 1;
                }
                axis[i + 1] = keyelement;
            }
        }

        int addCounter;
        public override void AddEntity(IBroadphaseEntity body)
        {
            bodyList.Add(body);

            axis1.Add(new SweepPoint(body, true, 0)); axis1.Add(new SweepPoint(body, false, 0));
            axis2.Add(new SweepPoint(body, true, 1)); axis2.Add(new SweepPoint(body, false, 1));
            axis3.Add(new SweepPoint(body, true, 2)); axis3.Add(new SweepPoint(body, false, 2));

            addCounter++;
        }

        Stack<OverlapPair> depricated = new Stack<OverlapPair>();
        public override bool RemoveEntity(IBroadphaseEntity body)
        {
            int count;

            count = 0;
            for (var i = 0; i < axis1.Count; i++)
            { if (axis1[i].Body == body) { count++; axis1.RemoveAt(i); if (count == 2) break; i--; } }

            count = 0;
            for (var i = 0; i < axis2.Count; i++)
            { if (axis2[i].Body == body) { count++; axis2.RemoveAt(i); if (count == 2) break; i--; } }

            count = 0;
            for (var i = 0; i < axis3.Count; i++)
            { if (axis3[i].Body == body) { count++; axis3.RemoveAt(i); if (count == 2) break; i--; } }

            foreach (var pair in fullOverlaps) if (pair.Entity1 == body || pair.Entity2 == body) depricated.Push(pair);
            while (depricated.Count > 0) fullOverlaps.Remove(depricated.Pop());

            bodyList.Remove(body);

            return true;
        }

        bool swapOrder;

        /// <summary>
        /// Tells the collisionsystem to check all bodies for collisions. Hook into the
        /// <see cref="CollisionSystem.PassedBroadphase"/>
        /// and <see cref="CollisionSystem.CollisionDetected"/> events to get the results.
        /// </summary>
        /// <param name="multiThreaded">If true internal multithreading is used.</param>
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

                {
                    sortCallback(axis1);
                    sortCallback(axis2);
                    sortCallback(axis3);
                }
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

        private void SortCallback(object obj)
        {
            SortAxis(obj as List<SweepPoint>);
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
            body = null; normal = default; fraction = float.MaxValue;

            var result = false;

            // TODO: This can be done better in CollisionSystemPersistenSAP
            foreach (var e in bodyList)
            {
                {
                    var b = e as RigidBody;

                    if (Raycast(b, rayOrigin, rayDirection, out var tempNormal, out var tempFraction))
                    {
                        if (tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
                        {
                            body = b;
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

            if (!body.BoundingBox.RayIntersect(ref rayOrigin, ref rayDirection)) return false;

            if (body.Shape is Multishape)
            {
                var ms = (body.Shape as Multishape).RequestWorkingClone();

                var multiShapeCollides = false;

                var transformedOrigin = rayOrigin - body.position;
                transformedOrigin = JVectorExtensions.Transform(transformedOrigin, body.invOrientation);
                Vector3 transformedDirection;
                transformedDirection = JVectorExtensions.Transform(rayDirection, body.invOrientation);

                var msLength = ms.Prepare(ref transformedOrigin, ref transformedDirection);

                for (var i = 0; i < msLength; i++)
                {
                    ms.SetCurrentShape(i);

                    if (GJKCollide.Raycast(ms, ref body.orientation, ref body.invOrientation, ref body.position,
                        ref rayOrigin, ref rayDirection, out var tempFraction, out var tempNormal))
                    {
                        if (tempFraction < fraction)
                        {
                            if (useTriangleMeshNormal && ms is TriangleMeshShape shape)
                            {
                                shape.CollisionNormal(out tempNormal);
                                tempNormal = JVectorExtensions.Transform(tempNormal, body.orientation);
                                tempNormal = -tempNormal;
                            }

                            normal = tempNormal;
                            fraction = tempFraction;
                            multiShapeCollides = true;
                        }
                    }
                }

                ms.ReturnWorkingClone();
                return multiShapeCollides;
            }
            else
            {
                return GJKCollide.Raycast(body.Shape, ref body.orientation, ref body.invOrientation, ref body.position,
                    ref rayOrigin, ref rayDirection, out fraction, out normal);
            }


        }
    }
}
