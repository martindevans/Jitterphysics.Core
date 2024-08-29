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

using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using System.Diagnostics;
using System.Numerics;

namespace Jitter.Collision
{
    /// <summary>
    /// A delegate for collision detection.
    /// </summary>
    /// <param name="body1">The first body colliding with the second one.</param>
    /// <param name="body2">The second body colliding with the first one.</param>
    /// <param name="point">The point on body in world coordinates, where collision occur.</param>
    /// <param name="normal">The normal pointing from body2 to body1.</param>
    /// <param name="penetration">Estimated penetration depth of the collision.</param>
    /// <seealso cref="CollisionSystem.Detect(bool)"/>
    /// <seealso cref="CollisionSystem.Detect(RigidBody,RigidBody)"/>
    public delegate void CollisionDetectedHandler(RigidBody body1,RigidBody body2, 
                    Vector3 point1, Vector3 point2, Vector3 normal,float penetration);

    /// <summary>
    /// A delegate to inform the user that a pair of bodies passed the broadsphase
    /// system of the engine.
    /// </summary>
    /// <param name="entity1">The first body.</param>
    /// <param name="entity2">The second body.</param>
    /// <returns>If false is returned the collision information is dropped. The CollisionDetectedHandler
    /// is never called.</returns>
    public delegate bool PassedBroadphaseHandler(RigidBody entity1, RigidBody entity2);

    /// <summary>
    /// A delegate to inform the user that a pair of bodies passed the narrowphase
    /// system of the engine.
    /// </summary>
    /// <param name="body1">The first body.</param>
    /// <param name="body2">The second body.</param>
    /// <returns>If false is returned the collision information is dropped. The CollisionDetectedHandler
    /// is never called.</returns>
    public delegate bool PassedNarrowphaseHandler(RigidBody body1, RigidBody body2, ref Vector3 point, ref Vector3 normal,float penetration);

    /// <summary>
    /// A delegate for raycasting.
    /// </summary>
    /// <param name="body">The body for which collision with the ray is detected.</param>
    /// <param name="normal">The normal of the collision.</param>
    /// <param name="fraction">The fraction which gives information where at the 
    /// ray the collision occured. The hitPoint is calculated by: rayStart+friction*direction.</param>
    /// <returns>If false is returned the collision information is dropped.</returns>
    public delegate bool RaycastCallback(RigidBody body,Vector3 normal, float fraction);

    /// <summary>
    /// CollisionSystem. Used by the world class to detect all collisions. 
    /// Can be used seperatly from the physics.
    /// </summary>
    public abstract class CollisionSystem
    {
        /// <summary>
        /// Remove a body from the collision system. Removing a body from the world
        /// does automatically remove it from the collision system.
        /// </summary>
        /// <param name="body">The body to remove.</param>
        /// <returns>Returns true if the body was successfully removed, otherwise false.</returns>
        public abstract bool RemoveEntity(RigidBody body);

        /// <summary>
        /// Add a body to the collision system. Adding a body to the world
        /// does automatically add it to the collision system.
        /// </summary>
        /// <param name="body">The body to remove.</param>
        public abstract void AddEntity(RigidBody body);

        /// <summary>
        /// Gets called when the broadphase system has detected possible collisions.
        /// </summary>
        public event PassedBroadphaseHandler PassedBroadphase;

        /// <summary>
        /// Gets called when broad- and narrow phase collision were positive.
        /// </summary>
        public event CollisionDetectedHandler CollisionDetected;

        public bool EnableSpeculativeContacts { get; set; }

        /// <summary>
        /// Initializes a new instance of the CollisionSystem.
        /// </summary>
        public CollisionSystem()
        {
        }

        /// <summary>
        /// If set to true the collision system uses the normal of
        /// the current colliding triangle as collision normal. This
        /// fixes unwanted behavior on triangle transitions.
        /// </summary>
        public bool UseTriangleMeshNormal { get; set; } = true;

        /// <summary>
        /// Checks two bodies for collisions using narrowphase.
        /// </summary>
        /// <param name="entity1">The first body.</param>
        /// <param name="entity2">The second body.</param>
        public virtual void Detect(RigidBody entity1, RigidBody entity2)
        {
            Debug.Assert(entity1 != entity2, "CollisionSystem reports selfcollision. Something is wrong.");

            if (entity1 != null)
            { 
                if(entity2 != null)
                {
                    // most common
                    DetectRigidRigid(entity1, entity2);
                }
            }
        }

        private void DetectRigidRigid(RigidBody body1, RigidBody body2)
        {
            var b1IsMulti = body1.Shape is Multishape;
            var b2IsMulti = body2.Shape is Multishape;

            var speculative = EnableSpeculativeContacts || body1.EnableSpeculativeContacts || body2.EnableSpeculativeContacts;

            Vector3 point, normal;
            float penetration;

            if (!b1IsMulti && !b2IsMulti)
            {
                if (XenoCollide.Detect(body1.Shape, body2.Shape, ref body1.orientation,
                    ref body2.orientation, ref body1.position, ref body2.position,
                    out point, out normal, out penetration))
                {
                    FindSupportPoints(body1, body2, body1.Shape, body2.Shape, ref point, ref normal, out var point1, out var point2);
                    RaiseCollisionDetected(body1, body2, ref point1, ref point2, ref normal, penetration);
                }
                else if (speculative)
                {
                    if (GJKCollide.ClosestPoints(body1.Shape, body2.Shape, ref body1.orientation, ref body2.orientation,
                            ref body1.position, ref body2.position, out var hit1, out var hit2, out normal))
                    {
                        var delta = hit2 - hit1;

                        if (delta.LengthSquared() < (body1.sweptDirection - body2.sweptDirection).LengthSquared())
                        {
                            penetration = Vector3.Dot(delta, normal);

                            if (penetration < 0.0f)
                            {
                                RaiseCollisionDetected(body1, body2, ref hit1, ref hit2, ref normal, penetration);
                            }

                        }
                    }

                }
            }
            else if (b1IsMulti && b2IsMulti)
            {
                var ms1 = (Multishape)body1.Shape;
                var ms2 = (Multishape)body2.Shape;

                ms1 = ms1.RequestWorkingClone();
                ms2 = ms2.RequestWorkingClone();

                var transformedBoundingBox = body2.boundingBox;
                transformedBoundingBox.InverseTransform(ref body1.position, ref body1.orientation);

                var ms1Length = ms1.Prepare(ref transformedBoundingBox);

                transformedBoundingBox = body1.boundingBox;
                transformedBoundingBox.InverseTransform(ref body2.position, ref body2.orientation);

                var ms2Length = ms2.Prepare(ref transformedBoundingBox);

                if (ms1Length == 0 || ms2Length == 0)
                {
                    ms1.ReturnWorkingClone();
                    ms2.ReturnWorkingClone();
                    return;
                }

                for (var i = 0; i < ms1Length; i++)
                {
                    ms1.SetCurrentShape(i);

                    for (var e = 0; e < ms2Length; e++)
                    {
                        ms2.SetCurrentShape(e);

                        if (XenoCollide.Detect(ms1, ms2, ref body1.orientation,
                            ref body2.orientation, ref body1.position, ref body2.position,
                            out point, out normal, out penetration))
                        {
                            FindSupportPoints(body1, body2, ms1, ms2, ref point, ref normal, out var point1, out var point2);
                            RaiseCollisionDetected(body1, body2, ref point1, ref point2, ref normal, penetration);
                        }
                        else if (speculative)
                        {
                            if (GJKCollide.ClosestPoints(ms1, ms2, ref body1.orientation, ref body2.orientation,
                                    ref body1.position, ref body2.position, out var hit1, out var hit2, out normal))
                            {
                                var delta = hit2 - hit1;

                                if (delta.LengthSquared() < (body1.sweptDirection - body2.sweptDirection).LengthSquared())
                                {
                                    penetration = Vector3.Dot(delta, normal);

                                    if (penetration < 0.0f)
                                    {
                                        RaiseCollisionDetected(body1, body2, ref hit1, ref hit2, ref normal, penetration);
                                    }
                                }
                            }


                        }
                    }
                }

                ms1.ReturnWorkingClone();
                ms2.ReturnWorkingClone();

            }
            else
            {
                RigidBody b1, b2;

                if (body2.Shape is Multishape) { b1 = body2; b2 = body1; }
                else { b2 = body2; b1 = body1; }

                var ms = (Multishape)b1.Shape;

                ms = ms.RequestWorkingClone();

                var transformedBoundingBox = b2.boundingBox;
                transformedBoundingBox.InverseTransform(ref b1.position, ref b1.orientation);

                var msLength = ms.Prepare(ref transformedBoundingBox);

                if (msLength == 0)
                {
                    ms.ReturnWorkingClone();
                    return;
                }

                for (var i = 0; i < msLength; i++)
                {
                    ms.SetCurrentShape(i);

                    if (XenoCollide.Detect(ms, b2.Shape, ref b1.orientation,
                        ref b2.orientation, ref b1.position, ref b2.position,
                        out point, out normal, out penetration))
                    {
                        FindSupportPoints(b1, b2, ms, b2.Shape, ref point, ref normal, out var point1, out var point2);

                        if (UseTriangleMeshNormal && ms is TriangleMeshShape)
                        {
                            (ms as TriangleMeshShape).CollisionNormal(out normal);
                            normal = normal.Transform(b1.orientation);
                        }

                        RaiseCollisionDetected(b1, b2, ref point1, ref point2, ref normal, penetration);
                    }
                    else if (speculative)
                    {
                        if (GJKCollide.ClosestPoints(ms, b2.Shape, ref b1.orientation, ref b2.orientation,
                                ref b1.position, ref b2.position, out var hit1, out var hit2, out normal))
                        {
                            var delta = hit2 - hit1;

                            if (delta.LengthSquared() < (body1.sweptDirection - body2.sweptDirection).LengthSquared())
                            {
                                penetration = Vector3.Dot(delta, normal);

                                if (penetration < 0.0f)
                                {
                                    RaiseCollisionDetected(b1, b2, ref hit1, ref hit2, ref normal, penetration);
                                }
                            }
                        }
                    }
                }

                ms.ReturnWorkingClone();
            }
        }

        private void FindSupportPoints(RigidBody body1, RigidBody body2,
            BaseShape shape1, BaseShape shape2, ref Vector3 point, ref Vector3 normal,
            out Vector3 point1, out Vector3 point2)
        {
            var mn = -normal;

            SupportMapping(body1, shape1, ref mn, out var sA);
            SupportMapping(body2, shape2, ref normal, out var sB);

            sA -= point;
            sB -= point;

            var dot1 = Vector3.Dot(sA, normal);
            var dot2 = Vector3.Dot(sB, normal);

            sA = normal * dot1;
            sB = normal * dot2;

            point1 = point + sA;
            point2 = point + sB;
        }

        private void SupportMapping(RigidBody body, BaseShape workingShape, ref Vector3 direction, out Vector3 result)
        {
            result = direction.Transform(body.invOrientation);
            result = workingShape.SupportMapping(result);
            result = result.Transform(body.orientation);
            result += body.position;
        }

        /// <summary>
        /// Sends a ray (definied by start and direction) through the scene (all bodies added).
        /// NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        public abstract bool Raycast(Vector3 rayOrigin, Vector3 rayDirection, RaycastCallback raycast, out RigidBody body, out Vector3 normal,out float fraction);

        /// <summary>
        /// Raycasts a single body. NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        public abstract bool Raycast(RigidBody body, Vector3 rayOrigin, Vector3 rayDirection, out Vector3 normal, out float fraction);


        /// <summary>
        /// Checks the state of two bodies.
        /// </summary>
        /// <param name="entity1">The first body.</param>
        /// <param name="entity2">The second body.</param>
        /// <returns>Returns true if both are static or inactive.</returns>
        public bool CheckBothStaticOrInactive(RigidBody entity1, RigidBody entity2)
        {
            return entity1.IsStaticOrInactive && entity2.IsStaticOrInactive;
       }

        /// <summary>
        /// Checks the AABB of the two rigid bodies.
        /// </summary>
        /// <param name="entity1">The first body.</param>
        /// <param name="entity2">The second body.</param>
        /// <returns>Returns true if an intersection occours.</returns>
        public bool CheckBoundingBoxes(RigidBody entity1, RigidBody entity2)
        {
            var box1 = entity1.BoundingBox;
            var box2 = entity2.BoundingBox;

            return box1.Max.Z >= box2.Min.Z && box1.Min.Z <= box2.Max.Z &&
                   box1.Max.Y >= box2.Min.Y && box1.Min.Y <= box2.Max.Y &&
                   box1.Max.X >= box2.Min.X && box1.Min.X <= box2.Max.X;
        }

        /// <summary>
        /// Raises the PassedBroadphase event.
        /// </summary>
        /// <param name="entity1">The first body.</param>
        /// <param name="entity2">The second body.</param>
        /// <returns>Returns false if the collision information
        /// should be dropped</returns>
        public bool RaisePassedBroadphase(RigidBody entity1, RigidBody entity2)
        {
            if (PassedBroadphase != null)
                return PassedBroadphase(entity1, entity2);

            // allow this detection by default
            return true;
        }


        /// <summary>
        /// Raises the CollisionDetected event.
        /// </summary>
        /// <param name="body1">The first body involved in the collision.</param>
        /// <param name="body2">The second body involved in the collision.</param>
        /// <param name="point">The collision point.</param>
        /// <param name="normal">The normal pointing to body1.</param>
        /// <param name="penetration">The penetration depth.</param>
        protected void RaiseCollisionDetected(RigidBody body1, RigidBody body2,
                                            ref Vector3 point1, ref Vector3 point2,
                                            ref Vector3 normal, float penetration)
        {
            CollisionDetected?.Invoke(body1, body2, point1, point2, normal, penetration);
        }

        /// <summary>
        /// Tells the collisionsystem to check all bodies for collisions. Hook into the <see cref="PassedBroadphase"/>
        /// and <see cref="CollisionDetected"/> events to get the results.
        /// </summary>
        /// <param name="multiThreaded">If true internal multithreading is used.</param>
        public abstract void Detect();
    }
}
