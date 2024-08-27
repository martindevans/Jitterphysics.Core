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

using Jitter.LinearMath;

namespace Jitter.Dynamics.Constraints.SingleBody
{

    public class PointOnPoint : Constraint
    {
        private JVector localAnchor1;
        private JVector anchor;

        private JVector r1;

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
        public PointOnPoint(RigidBody body, JVector localAnchor)
            : base(body, null)
        {
            localAnchor1 = localAnchor;

            anchor = body.position + JVectorExtensions.Transform(localAnchor, body.orientation);
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
        public JVector Anchor { get => anchor;
            set => anchor = value;
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

        JVector[] jacobian = new JVector[2];

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The 5simulation timestep</param>
        public override void PrepareForIteration(float timestep)
        {
            r1 = JVectorExtensions.Transform(localAnchor1, body1.orientation);
            var p1 = body1.position + r1;

            var dp = p1 - anchor;
            var deltaLength = dp.Length();

            var n = anchor - p1;
            if (n.LengthSquared() != 0.0f) n = JVector.Normalize(n);

            jacobian[0] = -1.0f * n;
            jacobian[1] = -1.0f * JVector.Cross(r1, n);

            effectiveMass = body1.inverseMass + JVector.Dot(JVectorExtensions.Transform(jacobian[1], body1.invInertiaWorld), jacobian[1]);

            softnessOverDt = softness / timestep;
            effectiveMass += softnessOverDt;

            effectiveMass = 1.0f / effectiveMass;

            bias = deltaLength * biasFactor * (1.0f / timestep);

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * accumulatedImpulse * jacobian[0];
                body1.angularVelocity += JVectorExtensions.Transform(accumulatedImpulse * jacobian[1], body1.invInertiaWorld);
            }
        }

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            var jv =
                JVector.Dot(body1.linearVelocity, jacobian[0]) +
                JVector.Dot(body1.angularVelocity, jacobian[1]);

            var softnessScalar = accumulatedImpulse * softnessOverDt;

            var lambda = -effectiveMass * (jv + bias + softnessScalar);

            accumulatedImpulse += lambda;

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * lambda * jacobian[0];
                body1.angularVelocity += JVectorExtensions.Transform(lambda * jacobian[1], body1.invInertiaWorld);
            }
        }

        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawPoint(anchor);
        }

    }

}
