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

using Jitter.LinearMath;

namespace Jitter.Dynamics.Constraints
{
    // Constraint formulation:
    // 
    // C = |(p1-p2) x l|
    //

    /// <summary>
    /// Constraints a point on a body to be fixed on a line
    /// which is fixed on another body.
    /// </summary>
    public class PointOnLine : Constraint
    {
        private JVector lineNormal;

        private JVector localAnchor1, localAnchor2;
        private JVector r1, r2;

        private float biasFactor = 0.5f;
        private float softness;

        /// <summary>
        /// Constraints a point on a body to be fixed on a line
        /// which is fixed on another body.
        /// </summary>
        /// <param name="body1"></param>
        /// <param name="body2"></param>
        /// <param name="lineStartPointBody1"></param>
        /// <param name="lineDirection"></param>
        /// <param name="pointBody2"></param>
        public PointOnLine(RigidBody body1, RigidBody body2,
            JVector lineStartPointBody1, JVector pointBody2) : base(body1,body2)
        {

            localAnchor1 = lineStartPointBody1 - body1.position;
            localAnchor2 = pointBody2 - body2.position;

            localAnchor1 = JVectorExtensions.Transform(localAnchor1, body1.invOrientation);
            localAnchor2 = JVectorExtensions.Transform(localAnchor2, body2.invOrientation);

            lineNormal = JVector.Normalize(lineStartPointBody1 - pointBody2);
        }

        public float AppliedImpulse => accumulatedImpulse;

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

        float effectiveMass;
        float accumulatedImpulse;
        float bias;
        float softnessOverDt;

        JVector[] jacobian = new JVector[4];

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The simulation timestep</param>
        public override void PrepareForIteration(float timestep)
        {
            r1 = JVectorExtensions.Transform(localAnchor1, body1.orientation);
            r2 = JVectorExtensions.Transform(localAnchor2, body2.orientation);

            JVector dp;
            var p1 = body1.position + r1;
            var p2 = body2.position + r2;

            dp = p2 - p1;

            var l = JVectorExtensions.Transform(lineNormal, body1.orientation);
            l = JVector.Normalize(l);

            var t = JVector.Cross(p1 - p2, l);
            if(t.LengthSquared() != 0.0f) t = JVector.Normalize(t);
            t = JVector.Cross(t, l);

            jacobian[0] = t;                      // linearVel Body1
            jacobian[1] = JVector.Cross(r1 + p2 - p1, t);     // angularVel Body1
            jacobian[2] = -1.0f * t;              // linearVel Body2
            jacobian[3] = -1.0f * JVector.Cross(r2, t);         // angularVel Body2

            effectiveMass = body1.inverseMass + body2.inverseMass
                + JVector.Dot(JVectorExtensions.Transform(jacobian[1], body1.invInertiaWorld), jacobian[1])
                                              + JVector.Dot(JVectorExtensions.Transform(jacobian[3], body2.invInertiaWorld), jacobian[3]);

            softnessOverDt = softness / timestep;
            effectiveMass += softnessOverDt;

            if(effectiveMass != 0) effectiveMass = 1.0f / effectiveMass;

            bias = -JVector.Cross(l, p2 - p1).Length() * biasFactor * (1.0f / timestep);

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * accumulatedImpulse * jacobian[0];
                body1.angularVelocity += JVectorExtensions.Transform(accumulatedImpulse * jacobian[1], body1.invInertiaWorld);
            }

            if (!body2.isStatic)
            {
                body2.linearVelocity += body2.inverseMass * accumulatedImpulse * jacobian[2];
                body2.angularVelocity += JVectorExtensions.Transform(accumulatedImpulse * jacobian[3], body2.invInertiaWorld);
            }
        }

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            var jv =
                JVector.Dot(body1.linearVelocity, jacobian[0]) +
                JVector.Dot(body1.angularVelocity, jacobian[1]) +
                JVector.Dot(body2.linearVelocity, jacobian[2]) +
                JVector.Dot(body2.angularVelocity, jacobian[3]);

            var softnessScalar = accumulatedImpulse * softnessOverDt;

            var lambda = -effectiveMass * (jv + bias + softnessScalar);

            accumulatedImpulse += lambda;

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * lambda * jacobian[0];
                body1.angularVelocity += JVectorExtensions.Transform(lambda * jacobian[1], body1.invInertiaWorld);
            }

            if (!body2.isStatic)
            {
                body2.linearVelocity += body2.inverseMass * lambda * jacobian[2];
                body2.angularVelocity += JVectorExtensions.Transform(lambda * jacobian[3], body2.invInertiaWorld);
            }
        }

        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawLine(body1.position + r1,
                body1.position + r1 + JVectorExtensions.Transform(lineNormal, body1.orientation) * 100.0f);
        }

    }
}
