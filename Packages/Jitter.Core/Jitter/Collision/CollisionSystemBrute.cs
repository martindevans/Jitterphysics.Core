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
    /// O(n^2) Broadphase detection. Every body is checked against each other body.
    /// This is pretty fast for scene containing just a few(~30) bodies.
    /// </summary>
    public class CollisionSystemBrute
        : CollisionSystem
    {
        private readonly List<RigidBody> bodyList = new();

        /// <summary>
        /// Remove a body from the collision system. Removing a body from the world
        /// does automatically remove it from the collision system.
        /// </summary>
        /// <param name="body">The body to remove.</param>
        /// <returns>Returns true if the body was successfully removed, otherwise false.</returns>
        public override bool RemoveEntity(RigidBody body)
        {
            // just keep our internal list in sync
            return bodyList.Remove(body);
        }

        /// <summary>
        /// Add a body to the collision system. Adding a body to the world
        /// does automatically add it to the collision system.
        /// </summary>
        /// <param name="body">The body to remove.</param>
        public override void AddEntity(RigidBody body)
        {
            if (bodyList.Contains(body))
                throw new ArgumentException("The body was already added to the collision system.", nameof(body));

            // just keep our internal list in sync
            bodyList.Add(body);
        }



        /// <summary>
        /// Tells the collisionsystem to check all bodies for collisions. Hook into the 
        /// <see cref="CollisionSystem.PassedBroadphase"/>
        /// and <see cref="CollisionSystem.CollisionDetected"/> events to get the results.
        /// </summary>
        public override void Detect()
        {
            var count = bodyList.Count;

            {
                for (var i = 0; i < count; i++)
                {
                    for (var e = i + 1; e < count; e++)
                    {
                        if (!CheckBothStaticOrInactive(bodyList[i], bodyList[e]) && CheckBoundingBoxes(bodyList[i], bodyList[e]))
                        {
                            if (RaisePassedBroadphase(bodyList[i], bodyList[e]))
                            {
                                if (swapOrder) Detect(bodyList[i], bodyList[e]);
                                else Detect(bodyList[e], bodyList[i]);
                                swapOrder = !swapOrder;
                            }
                        }
                    }
                }
            }
        }


        private bool swapOrder;

        /// <summary>
        /// Sends a ray (definied by start and direction) through the scene (all bodies added).
        /// NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        public override bool Raycast(Vector3 rayOrigin, Vector3 rayDirection, RaycastCallback raycast, out RigidBody body, out Vector3 normal, out float fraction)
        {
            body = null;
            normal = default;
            fraction = float.MaxValue;

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
            fraction = float.MaxValue; normal = default;

            if (!body.BoundingBox.RayIntersect(rayOrigin, rayDirection)) return false;

            if (body.Shape is Multishape)
            {
                var ms = (body.Shape as Multishape).RequestWorkingClone();

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
                            if (UseTriangleMeshNormal && ms is TriangleMeshShape)
                            {
                                (ms as TriangleMeshShape).CollisionNormal(out tempNormal);
                                tempNormal = tempNormal.Transform(body.orientation);
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
                return GJKCollide.Raycast(body.Shape, ref body.orientation, ref body.position,
                    ref rayOrigin, ref rayDirection, out fraction, out normal);
            }


        }
    }
}
