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

using System.Numerics;
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
    public class PointOnLine
        : BaseConstraint
    {
        private Vector3 lineNormal;

        private Vector3 localAnchor1, localAnchor2;
        private Vector3 r1, r2;

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
            Vector3 lineStartPointBody1, Vector3 pointBody2) : base(body1,body2)
        {

            localAnchor1 = lineStartPointBody1 - body1.position;
            localAnchor2 = pointBody2 - body2.position;

            localAnchor1 = localAnchor1.Transform(body1.invOrientation);
            localAnchor2 = localAnchor2.Transform(body2.invOrientation);

            lineNormal = Vector3.Normalize(lineStartPointBody1 - pointBody2);
        }

        public float AppliedImpulse { get; private set; }

        /// <summary>
        /// Defines how big the applied impulses can get.
        /// </summary>
        public float Softness { get; set; }

        /// <summary>
        /// Defines how big the applied impulses can get which correct errors.
        /// </summary>
        public float BiasFactor { get; set; } = 0.5f;

        private float effectiveMass;
        private float bias;
        private float softnessOverDt;

        private readonly Vector3[] jacobian = new Vector3[4];

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The simulation timestep</param>
        public override void PrepareForIteration(float timestep)
        {
            r1 = localAnchor1.Transform(Body1.orientation);
            r2 = localAnchor2.Transform(Body2.orientation);

            Vector3 dp;
            var p1 = Body1.position + r1;
            var p2 = Body2.position + r2;

            dp = p2 - p1;

            var l = lineNormal.Transform(Body1.orientation);
            l = Vector3.Normalize(l);

            var t = Vector3.Cross(p1 - p2, l);
            if(t.LengthSquared() != 0.0f) t = Vector3.Normalize(t);
            t = Vector3.Cross(t, l);

            jacobian[0] = t;                      // linearVel Body1
            jacobian[1] = Vector3.Cross(r1 + p2 - p1, t);     // angularVel Body1
            jacobian[2] = -1.0f * t;              // linearVel Body2
            jacobian[3] = -1.0f * Vector3.Cross(r2, t);         // angularVel Body2

            effectiveMass = Body1.inverseMass + Body2.inverseMass
                + Vector3.Dot(jacobian[1].Transform(Body1.invInertiaWorld), jacobian[1])
                                              + Vector3.Dot(jacobian[3].Transform(Body2.invInertiaWorld), jacobian[3]);

            softnessOverDt = Softness / timestep;
            effectiveMass += softnessOverDt;

            if(effectiveMass != 0) effectiveMass = 1.0f / effectiveMass;

            bias = -Vector3.Cross(l, p2 - p1).Length() * BiasFactor * (1.0f / timestep);

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

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            var jv =
                Vector3.Dot(Body1.linearVelocity, jacobian[0]) +
                Vector3.Dot(Body1.angularVelocity, jacobian[1]) +
                Vector3.Dot(Body2.linearVelocity, jacobian[2]) +
                Vector3.Dot(Body2.angularVelocity, jacobian[3]);

            var softnessScalar = AppliedImpulse * softnessOverDt;

            var lambda = -effectiveMass * (jv + bias + softnessScalar);

            AppliedImpulse += lambda;

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
            drawer.DrawLine(Body1.position + r1,
                Body1.position + r1 + lineNormal.Transform(Body1.orientation) * 100.0f);
        }

    }
}
