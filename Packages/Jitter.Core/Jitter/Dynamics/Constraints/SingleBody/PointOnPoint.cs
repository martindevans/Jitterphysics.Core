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

namespace Jitter.Dynamics.Constraints.SingleBody
{

    public class PointOnPoint : BaseConstraint
    {
        private Vector3 localAnchor1;
        private Vector3 anchor;

        private Vector3 r1;

        private float biasFactor = 0.1f;
        private float softness = 0.01f;

        /// <summary>
        /// Initializes a new instance of the DistanceConstraint class.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <param name="anchor1">The anchor point of the first body in world space. 
        /// The distance is given by the initial distance between both anchor points.</param>
        /// <param name="anchor2">The anchor point of the second body in world space.
        /// The distance is given by the initial distance between both anchor points.</param>
        public PointOnPoint(RigidBody body, Vector3 localAnchor)
            : base(body, null)
        {
            localAnchor1 = localAnchor;

            anchor = body.position + localAnchor.Transform(body.orientation);
        }

        public float AppliedImpulse => accumulatedImpulse;

        /// <summary>
        /// Defines how big the applied impulses can get.
        /// </summary>
        public float Softness { get => softness;
            set => softness = value;
        }

        /// <summary>
        /// The anchor point in the world.
        /// </summary>
        public Vector3 Anchor { get => anchor;
            set => anchor = value;
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

        private Vector3[] jacobian = new Vector3[2];

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The 5simulation timestep</param>
        public override void PrepareForIteration(float timestep)
        {
            r1 = localAnchor1.Transform(Body1.orientation);
            var p1 = Body1.position + r1;

            var dp = p1 - anchor;
            var deltaLength = dp.Length();

            var n = anchor - p1;
            if (n.LengthSquared() != 0.0f) n = Vector3.Normalize(n);

            jacobian[0] = -1.0f * n;
            jacobian[1] = -1.0f * Vector3.Cross(r1, n);

            effectiveMass = Body1.inverseMass + Vector3.Dot(jacobian[1].Transform(Body1.invInertiaWorld), jacobian[1]);

            softnessOverDt = softness / timestep;
            effectiveMass += softnessOverDt;

            effectiveMass = 1.0f / effectiveMass;

            bias = deltaLength * biasFactor * (1.0f / timestep);

            if (!Body1.IsStatic)
            {
                Body1.linearVelocity += Body1.inverseMass * accumulatedImpulse * jacobian[0];
                Body1.angularVelocity += (accumulatedImpulse * jacobian[1]).Transform(Body1.invInertiaWorld);
            }
        }

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            var jv =
                Vector3.Dot(Body1.linearVelocity, jacobian[0]) +
                Vector3.Dot(Body1.angularVelocity, jacobian[1]);

            var softnessScalar = accumulatedImpulse * softnessOverDt;

            var lambda = -effectiveMass * (jv + bias + softnessScalar);

            accumulatedImpulse += lambda;

            if (!Body1.IsStatic)
            {
                Body1.linearVelocity += Body1.inverseMass * lambda * jacobian[0];
                Body1.angularVelocity += (lambda * jacobian[1]).Transform(Body1.invInertiaWorld);
            }
        }

        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawPoint(anchor);
        }

    }

}
