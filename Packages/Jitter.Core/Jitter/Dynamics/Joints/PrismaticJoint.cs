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
using Jitter.Dynamics.Constraints;

namespace Jitter.Dynamics.Joints
{
    public class PrismaticJoint
        : BaseJoint
    {
        public PointPointDistance MaximumDistanceConstraint { get; }
        public PointPointDistance MinimumDistanceConstraint { get; }

        public FixedAngle FixedAngleConstraint { get; }
        public PointOnLine PointOnLineConstraint { get; }

        public PrismaticJoint(World world, RigidBody body1, RigidBody body2)
            : base(world)
        {
            FixedAngleConstraint = new(body1, body2);
            PointOnLineConstraint = new(body1, body2, body1.position, body2.position);
        }

        public PrismaticJoint(World world, RigidBody body1, RigidBody body2, float minimumDistance, float maximumDistance)
            : base(world)
        {
            FixedAngleConstraint = new(body1, body2);
            PointOnLineConstraint = new(body1, body2, body1.position, body2.position);

            MinimumDistanceConstraint = new(body1, body2, body1.position, body2.position)
            {
                Behavior = PointPointDistance.DistanceBehavior.LimitMinimumDistance,
                Distance = minimumDistance,
            };

            MaximumDistanceConstraint = new(body1, body2, body1.position, body2.position)
            {
                Behavior = PointPointDistance.DistanceBehavior.LimitMaximumDistance,
                Distance = maximumDistance,
            };
        }


        public PrismaticJoint(World world, RigidBody body1, RigidBody body2, Vector3 pointOnBody1, Vector3 pointOnBody2)
            : base(world)
        {
            FixedAngleConstraint = new(body1, body2);
            PointOnLineConstraint = new(body1, body2, pointOnBody1, pointOnBody2);
        }

        /// <inheritdoc />
        public override void Activate()
        {
            if (MaximumDistanceConstraint != null)
                World.Add(MaximumDistanceConstraint);
            if (MinimumDistanceConstraint != null)
                World.Add(MinimumDistanceConstraint);

            World.Add(FixedAngleConstraint);
            World.Add(PointOnLineConstraint);
        }

        /// <inheritdoc />
        public override void Deactivate()
        {
            if (MaximumDistanceConstraint != null)
                World.Remove(MaximumDistanceConstraint);
            if (MinimumDistanceConstraint != null)
                World.Remove(MinimumDistanceConstraint);

            World.Remove(FixedAngleConstraint);
            World.Remove(PointOnLineConstraint);
        }
    }
}
