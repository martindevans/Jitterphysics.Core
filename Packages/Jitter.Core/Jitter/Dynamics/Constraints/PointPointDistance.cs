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

using System;
using System.Numerics;
using Jitter.LinearMath;

namespace Jitter.Dynamics.Constraints
{
    /// <summary>
    /// The distance between two given points on two bodies will not
    /// exceed a value.
    /// </summary>
    public class PointPointDistance
        : BaseConstraint
    {
        public enum DistanceBehavior
        {
            LimitDistance,
            LimitMaximumDistance,
            LimitMinimumDistance,
        }

        private Vector3 r1, r2;

        /// <summary>
        /// Initializes a new instance of the DistanceConstraint class.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <param name="anchor1">The anchor point of the first body in world space. 
        /// The distance is given by the initial distance between both anchor points.</param>
        /// <param name="anchor2">The anchor point of the second body in world space.
        /// The distance is given by the initial distance between both anchor points.</param>
        public PointPointDistance(RigidBody body1, RigidBody body2, Vector3 anchor1,Vector3 anchor2)
            : base(body1, body2)
        {
            LocalAnchor1 = anchor1 - body1.position;
            LocalAnchor2 = anchor2 - body2.position;

            LocalAnchor1 = LocalAnchor1.Transform(body1.invOrientation);
            LocalAnchor2 = LocalAnchor2.Transform(body2.invOrientation);

            Distance = (anchor1 - anchor2).Length();
        }

        public float AppliedImpulse { get; private set; }

        /// <summary>
        /// 
        /// </summary>
        public float Distance { get; set; }

        /// <summary>
        /// 
        /// </summary>
        public DistanceBehavior Behavior { get; set; } = DistanceBehavior.LimitDistance;

        /// <summary>
        /// The anchor point of body1 in local (body) coordinates.
        /// </summary>
        public Vector3 LocalAnchor1 { get; set; }

        /// <summary>
        /// The anchor point of body2 in local (body) coordinates.
        /// </summary>
        public Vector3 LocalAnchor2 { get; set; }

        /// <summary>
        /// Defines how big the applied impulses can get.
        /// </summary>
        public float Softness { get; set; } = 0.01f;

        /// <summary>
        /// Defines how big the applied impulses can get which correct errors.
        /// </summary>
        public float BiasFactor { get; set; } = 0.1f;

        private float effectiveMass;
        private float bias;
        private float softnessOverDt;

        private readonly Vector3[] jacobian = new Vector3[4];

        private bool skipConstraint;

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The 5simulation timestep</param>
        public override void PrepareForIteration(float timestep)
        {
            r1 = LocalAnchor1.Transform(Body1.orientation);
            r2 = LocalAnchor2.Transform(Body2.orientation);

            var p1 = Body1.position + r1;
            var p2 = Body2.position + r2;

            var dp = p2 - p1;

            var deltaLength = dp.Length() - Distance;

            if (Behavior == DistanceBehavior.LimitMaximumDistance && deltaLength <= 0.0f)
            {
                skipConstraint = true;
            }
            else if (Behavior == DistanceBehavior.LimitMinimumDistance && deltaLength >= 0.0f)
            {
                skipConstraint = true;
            }
            else
            {
                skipConstraint = false;

                var n = p2 - p1;
                if (n.LengthSquared() != 0.0f) n = Vector3.Normalize(n);

                jacobian[0] = -1.0f * n;
                jacobian[1] = -1.0f * Vector3.Cross(r1, n);
                jacobian[2] = 1.0f * n;
                jacobian[3] = Vector3.Cross(r2, n);

                effectiveMass = Body1.inverseMass + Body2.inverseMass
                    + Vector3.Dot(jacobian[1].Transform(Body1.invInertiaWorld), jacobian[1])
                                                  + Vector3.Dot(jacobian[3].Transform(Body2.invInertiaWorld), jacobian[3]);

                softnessOverDt = Softness / timestep;
                effectiveMass += softnessOverDt;

                effectiveMass = 1.0f / effectiveMass;

                bias = deltaLength * BiasFactor * (1.0f / timestep);

                if (!Body1.IsStatic)
                {
                    Body1.linearVelocity += Body1.inverseMass * AppliedImpulse * jacobian[0];
                    Body1.angularVelocity += (AppliedImpulse * jacobian[1]).Transform(Body1.invInertiaWorld);
                }

                if (!Body2.IsStatic)
                {
                    Body2.linearVelocity += Body2.inverseMass * AppliedImpulse * jacobian[2];
                    Body2.angularVelocity += (AppliedImpulse * jacobian[3]).Transform(Body2.invInertiaWorld);
                }
            }
            
        }

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            if (skipConstraint) return;

            var jv =
                Vector3.Dot(Body1.linearVelocity, jacobian[0]) +
                Vector3.Dot(Body1.angularVelocity, jacobian[1]) +
                Vector3.Dot(Body2.linearVelocity, jacobian[2]) +
                Vector3.Dot(Body2.angularVelocity, jacobian[3]);

            var softnessScalar = AppliedImpulse * softnessOverDt;

            var lambda = -effectiveMass * (jv + bias + softnessScalar);

            if (Behavior == DistanceBehavior.LimitMinimumDistance)
            {
                var previousAccumulatedImpulse = AppliedImpulse;
                AppliedImpulse = Math.Max(AppliedImpulse + lambda, 0);
                lambda = AppliedImpulse - previousAccumulatedImpulse;
            }
            else if (Behavior == DistanceBehavior.LimitMaximumDistance)
            {
                var previousAccumulatedImpulse = AppliedImpulse;
                AppliedImpulse = Math.Min(AppliedImpulse + lambda, 0);
                lambda = AppliedImpulse - previousAccumulatedImpulse;
            }
            else
            {
                AppliedImpulse += lambda;
            }

            if (!Body1.IsStatic)
            {
                Body1.linearVelocity += Body1.inverseMass * lambda * jacobian[0];
                Body1.angularVelocity += (lambda * jacobian[1]).Transform(Body1.invInertiaWorld);
            }

            if (!Body2.IsStatic)
            {
                Body2.linearVelocity += Body2.inverseMass * lambda * jacobian[2];
                Body2.angularVelocity += (lambda * jacobian[3]).Transform(Body2.invInertiaWorld);
            }
        }


        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawLine(Body1.position + r1, Body2.position + r2);
        }

    }
}
