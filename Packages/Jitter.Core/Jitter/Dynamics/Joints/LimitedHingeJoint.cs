﻿using System;
using System.Numerics;
using Jitter.LinearMath;
using Jitter.Dynamics.Constraints;

namespace Jitter.Dynamics.Joints
{

    /// <summary>
    /// Limited hinge joint.
    /// </summary>
    public class LimitedHingeJoint
        : BaseJoint
    {
        public PointOnPoint PointOnPointConstraint0 { get; }
        public PointOnPoint PointOnPointConstraint1 { get; }
        public PointPointDistance DistanceConstraint { get; }

        /// <summary>
        /// Initializes a new instance of the HingeJoint class.
        /// </summary>
        /// <param name="world">The world class where the constraints get added to.</param>
        /// <param name="body1">The first body connected to the second one.</param>
        /// <param name="body2">The second body connected to the first one.</param>
        /// <param name="position">The position in world space where both bodies get connected.</param>
        /// <param name="hingeAxis">The axis if the hinge.</param>
        /// <param name="hingeFwdAngle"></param>
        /// <param name="hingeBckAngle"></param>
        public LimitedHingeJoint(World world, RigidBody body1, RigidBody body2, Vector3 position, Vector3 hingeAxis, float hingeFwdAngle, float hingeBckAngle)
            : base(world)
        {
            // Create the hinge first, two point constraints

            hingeAxis *= 0.5f;

            var pos1 = position; pos1 += hingeAxis;
            var pos2 = position; pos2 -= hingeAxis;

            PointOnPointConstraint0 = new(body1, body2, pos1);
            PointOnPointConstraint1 = new(body1, body2, pos2);


            // Now the limit, one max distance constraint

            hingeAxis = Vector3.Normalize(hingeAxis);

            // choose a direction that is perpendicular to the hinge
            var perpDir = JVectorExtensions.Up;

            if (Vector3.Dot(perpDir, hingeAxis) > 0.1f) perpDir = JVectorExtensions.Right;

            // now make it perpendicular to the hinge
            var sideAxis = Vector3.Cross(hingeAxis, perpDir);
            perpDir = Vector3.Cross(sideAxis, hingeAxis);
            perpDir = Vector3.Normalize(perpDir);

            // the length of the "arm" TODO take this as a parameter? what's
            // the effect of changing it?
            const float len = 10.0f * 3;

            // Choose a position using that dir. this will be the anchor point
            // for body 0. relative to hinge
            var hingeRelAnchorPos0 = perpDir * len;


            // anchor point for body 2 is chosen to be in the middle of the
            // angle range.  relative to hinge
            var angleToMiddle = 0.5f * (hingeFwdAngle - hingeBckAngle);
            var hingeRelAnchorPos1 = hingeRelAnchorPos0.Transform(JMatrix.CreateFromAxisAngle(hingeAxis, -angleToMiddle / 360.0f * 2.0f * MathF.PI));

            // work out the "string" length
            var hingeHalfAngle = 0.5f * (hingeFwdAngle + hingeBckAngle);
            var allowedDistance = len * 2.0f * MathF.Sin(hingeHalfAngle * 0.5f / 360.0f * 2.0f * MathF.PI);

            var hingePos = body1.Position;
            var relPos0c = hingePos + hingeRelAnchorPos0;
            var relPos1c = hingePos + hingeRelAnchorPos1;

            DistanceConstraint = new(body1, body2, relPos0c, relPos1c)
            {
                Distance = allowedDistance,
                Behavior = PointPointDistance.DistanceBehavior.LimitMaximumDistance,
            };
        }

        /// <summary>
        /// Adds the internal constraints of this joint to the world class.
        /// </summary>
        public override void Activate()
        {
            World.Add(PointOnPointConstraint0);
            World.Add(PointOnPointConstraint1);
            World.Add(DistanceConstraint);
        }

        /// <summary>
        /// Removes the internal constraints of this joint from the world class.
        /// </summary>
        public override void Deactivate()
        {
            World.Remove(PointOnPointConstraint0);
            World.Remove(PointOnPointConstraint1);
            World.Remove(DistanceConstraint);
        }
    }
}
