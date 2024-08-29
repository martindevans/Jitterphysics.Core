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
    public class PointPointDistance : Constraint
    {
        public enum DistanceBehavior
        {
            LimitDistance,
            LimitMaximumDistance,
            LimitMinimumDistance,
        }

        private Vector3 localAnchor1, localAnchor2;
        private Vector3 r1, r2;

        private float biasFactor = 0.1f;
        private float softness = 0.01f;
        private float distance;

        private DistanceBehavior behavior = DistanceBehavior.LimitDistance;

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
            localAnchor1 = anchor1 - body1.position;
            localAnchor2 = anchor2 - body2.position;

            localAnchor1 = localAnchor1.Transform(body1.invOrientation);
            localAnchor2 = localAnchor2.Transform(body2.invOrientation);

            distance = (anchor1 - anchor2).Length();
        }

        public float AppliedImpulse => accumulatedImpulse;

        /// <summary>
        /// 
        /// </summary>
        public float Distance { get => distance;
            set => distance = value;
        }

        /// <summary>
        /// 
        /// </summary>
        public DistanceBehavior Behavior { get => behavior;
            set => behavior = value;
        }

        /// <summary>
        /// The anchor point of body1 in local (body) coordinates.
        /// </summary>
        public Vector3 LocalAnchor1 { get => localAnchor1;
            set => localAnchor1 = value;
        }

        /// <summary>
        /// The anchor point of body2 in local (body) coordinates.
        /// </summary>
        public Vector3 LocalAnchor2 { get => localAnchor2;
            set => localAnchor2 = value;
        }

        /// <summary>
        /// Defines how big the applied impulses can get.
        /// </summary>
        public float Softness { get => softness;
            set => softness = value;
        }

        /// <summary>
        /// Defines how big the applied impulses can get which correct errors.
        /// </summary>
        public float BiasFactor { get => biasFactor;
            set => biasFactor = value;
        }

        private float effectiveMass;
        private float accumulatedImpulse;
        private float bias;
        private float softnessOverDt;

        private Vector3[] jacobian = new Vector3[4];

        private bool skipConstraint;

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The 5simulation timestep</param>
        public override void PrepareForIteration(float timestep)
        {
            r1 = localAnchor1.Transform(body1.orientation);
            r2 = localAnchor2.Transform(body2.orientation);

            var p1 = body1.position + r1;
            var p2 = body2.position + r2;

            var dp = p2 - p1;

            var deltaLength = dp.Length() - distance;

            if (behavior == DistanceBehavior.LimitMaximumDistance && deltaLength <= 0.0f)
            {
                skipConstraint = true;
            }
            else if (behavior == DistanceBehavior.LimitMinimumDistance && deltaLength >= 0.0f)
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

                effectiveMass = body1.inverseMass + body2.inverseMass
                    + Vector3.Dot(jacobian[1].Transform(body1.invInertiaWorld), jacobian[1])
                                                  + Vector3.Dot(jacobian[3].Transform(body2.invInertiaWorld), jacobian[3]);

                softnessOverDt = softness / timestep;
                effectiveMass += softnessOverDt;

                effectiveMass = 1.0f / effectiveMass;

                bias = deltaLength * biasFactor * (1.0f / timestep);

                if (!body1.IsStatic)
                {
                    body1.linearVelocity += body1.inverseMass * accumulatedImpulse * jacobian[0];
                    body1.angularVelocity += (accumulatedImpulse * jacobian[1]).Transform(body1.invInertiaWorld);
                }

                if (!body2.IsStatic)
                {
                    body2.linearVelocity += body2.inverseMass * accumulatedImpulse * jacobian[2];
                    body2.angularVelocity += (accumulatedImpulse * jacobian[3]).Transform(body2.invInertiaWorld);
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
                Vector3.Dot(body1.linearVelocity, jacobian[0]) +
                Vector3.Dot(body1.angularVelocity, jacobian[1]) +
                Vector3.Dot(body2.linearVelocity, jacobian[2]) +
                Vector3.Dot(body2.angularVelocity, jacobian[3]);

            var softnessScalar = accumulatedImpulse * softnessOverDt;

            var lambda = -effectiveMass * (jv + bias + softnessScalar);

            if (behavior == DistanceBehavior.LimitMinimumDistance)
            {
                var previousAccumulatedImpulse = accumulatedImpulse;
                accumulatedImpulse = Math.Max(accumulatedImpulse + lambda, 0);
                lambda = accumulatedImpulse - previousAccumulatedImpulse;
            }
            else if (behavior == DistanceBehavior.LimitMaximumDistance)
            {
                var previousAccumulatedImpulse = accumulatedImpulse;
                accumulatedImpulse = Math.Min(accumulatedImpulse + lambda, 0);
                lambda = accumulatedImpulse - previousAccumulatedImpulse;
            }
            else
            {
                accumulatedImpulse += lambda;
            }

            if (!body1.IsStatic)
            {
                body1.linearVelocity += body1.inverseMass * lambda * jacobian[0];
                body1.angularVelocity += (lambda * jacobian[1]).Transform(body1.invInertiaWorld);
            }

            if (!body2.IsStatic)
            {
                body2.linearVelocity += body2.inverseMass * lambda * jacobian[2];
                body2.angularVelocity += (lambda * jacobian[3]).Transform(body2.invInertiaWorld);
            }
        }


        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawLine(body1.position + r1, body2.position + r2);
        }

    }
}
