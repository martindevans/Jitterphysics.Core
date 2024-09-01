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
using System.Numerics;
using Jitter.LinearMath;

namespace Jitter.Dynamics.Constraints.SingleBody
{
    // Constraint formulation:
    // 
    // C_1 = R1_x - R2_x
    // C_2 = ...
    // C_3 = ...
    //
    // Derivative:
    //
    // dC_1/dt = w1_x - w2_x
    // dC_2/dt = ...
    // dC_3/dt = ...
    //
    // Jacobian:
    // 
    // dC/dt = J*v+b
    //
    // v = (v1x v1y v1z w1x w1y w1z v2x v2y v2z w2x w2y w2z)^(T) 
    //
    //     v1x v1y v1z w1x w1y w1z v2x v2y v2z w2x w2y w2z
    //     -------------------------------------------------
    // J = 0   0   0   1   0    0   0   0   0   -1   0   0   <- dC_1/dt
    //     0   0   0   0   1    0   0   0   0    0  -1   0   <- ...  
    //     0   0   0   0   0    1   0   0   0    0   0  -1   <- ...
    //
    // Effective Mass:
    //
    // 1/m_eff = [J^T * M^-1 * J] = I1^(-1) + I2^(-1)

    /// <summary>
    /// The body stays at a fixed angle relative to
    /// world space.
    /// </summary>
    public class FixedAngle
        : BaseConstraint
    {
        private Vector3 accumulatedImpulse;

        /// <summary>
        /// Constraints two bodies to always have the same relative
        /// orientation to each other. Combine the AngleConstraint with a PointOnLine
        /// Constraint to get a prismatic joint.
        /// </summary>
        public FixedAngle(RigidBody body1)
            : base(body1, null)
        {
            InitialOrientation = body1.orientation;
        }

        /// <summary>
        /// Defines how big the applied impulses can get.
        /// </summary>
        public float Softness { get; set; }

        /// <summary>
        /// Defines how big the applied impulses can get which correct errors.
        /// </summary>
        public float BiasFactor { get; set; } = 0.05f;

        public JMatrix InitialOrientation { get; set; }

        private JMatrix effectiveMass;
        private Vector3 bias;
        private float softnessOverDt;

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The 5simulation timestep</param>
        public override void PrepareForIteration(float timestep)
        {
            effectiveMass = Body1.invInertiaWorld;

            softnessOverDt = Softness / timestep;

            effectiveMass.Diagonal += new Vector3(softnessOverDt);

            effectiveMass = effectiveMass.Inverse();
            var q = JMatrix.Transpose(InitialOrientation) * Body1.orientation;

            var x = q.M32 - q.M23;
            var y = q.M13 - q.M31;
            var z = q.M21 - q.M12;

            var r = MathF.Sqrt(x * x + y * y + z * z);
            var t = q.Trace;

            var angle = (float)Math.Atan2(r, t - 1);
            var axis = new Vector3(x, y, z) * angle;

            if (r != 0.0f) axis *= (1.0f / r);

            bias = axis * BiasFactor * (-1.0f / timestep);

            // Apply previous frame solution as initial guess for satisfying the constraint.
            if (!Body1.IsStatic)
                Body1.angularVelocity += accumulatedImpulse.Transform(Body1.invInertiaWorld);
        }

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            var jv = Body1.angularVelocity;

            var softnessVector = accumulatedImpulse * softnessOverDt;

            var lambda = -1.0f * (jv + bias + softnessVector).Transform(effectiveMass);

            accumulatedImpulse += lambda;

            if (!Body1.IsStatic)
                Body1.angularVelocity += lambda.Transform(Body1.invInertiaWorld);
        }

    }
}
