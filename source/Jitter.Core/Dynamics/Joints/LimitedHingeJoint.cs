using System;
using Jitter.LinearMath;
using Jitter.Dynamics.Constraints;

namespace Jitter.Dynamics.Joints
{

    /// <summary>
    /// Limited hinge joint.
    /// </summary>
    public class LimitedHingeJoint : Joint
    {


        private PointOnPoint[] worldPointConstraint;
        private PointPointDistance distance;

        public PointOnPoint PointConstraint1 => worldPointConstraint[0];
        public PointOnPoint PointConstraint2 => worldPointConstraint[1];

        public PointPointDistance DistanceConstraint => distance;


        /// <summary>
        /// Initializes a new instance of the HingeJoint class.
        /// </summary>
        /// <param name="world">The world class where the constraints get added to.</param>
        /// <param name="body1">The first body connected to the second one.</param>
        /// <param name="body2">The second body connected to the first one.</param>
        /// <param name="position">The position in world space where both bodies get connected.</param>
        /// <param name="hingeAxis">The axis if the hinge.</param>
        public LimitedHingeJoint(World world, RigidBody body1, RigidBody body2, JVector position, JVector hingeAxis,
            float hingeFwdAngle, float hingeBckAngle)
            : base(world)
        {
            // Create the hinge first, two point constraints

            worldPointConstraint = new PointOnPoint[2];

            hingeAxis *= 0.5f;

            var pos1 = position; pos1 = pos1 + hingeAxis;
            var pos2 = position; pos2 = pos2 - hingeAxis;

            worldPointConstraint[0] = new PointOnPoint(body1, body2, pos1);
            worldPointConstraint[1] = new PointOnPoint(body1, body2, pos2);


            // Now the limit, one max distance constraint

            hingeAxis = JVector.Normalize(hingeAxis);

            // choose a direction that is perpendicular to the hinge
            var perpDir = JVector.Up;

            if (JVector.Dot(perpDir, hingeAxis) > 0.1f) perpDir = JVector.Right;

            // now make it perpendicular to the hinge
            var sideAxis = JVector.Cross(hingeAxis, perpDir);
            perpDir = JVector.Cross(sideAxis, hingeAxis);
            perpDir = JVector.Normalize(perpDir);

            // the length of the "arm" TODO take this as a parameter? what's
            // the effect of changing it?
            var len = 10.0f * 3;

            // Choose a position using that dir. this will be the anchor point
            // for body 0. relative to hinge
            var hingeRelAnchorPos0 = perpDir * len;


            // anchor point for body 2 is chosen to be in the middle of the
            // angle range.  relative to hinge
            var angleToMiddle = 0.5f * (hingeFwdAngle - hingeBckAngle);
            var hingeRelAnchorPos1 = JVector.Transform(hingeRelAnchorPos0, JMatrix.CreateFromAxisAngle(hingeAxis, -angleToMiddle / 360.0f * 2.0f * MathF.PI));

            // work out the "string" length
            var hingeHalfAngle = 0.5f * (hingeFwdAngle + hingeBckAngle);
            var allowedDistance = len * 2.0f * (float)Math.Sin(hingeHalfAngle * 0.5f / 360.0f * 2.0f * MathF.PI);

            var hingePos = body1.Position;
            var relPos0c = hingePos + hingeRelAnchorPos0;
            var relPos1c = hingePos + hingeRelAnchorPos1;

            distance = new PointPointDistance(body1, body2, relPos0c, relPos1c);
            distance.Distance = allowedDistance;
            distance.Behavior = PointPointDistance.DistanceBehavior.LimitMaximumDistance;

        }


        /// <summary>
        /// Adds the internal constraints of this joint to the world class.
        /// </summary>
        public override void Activate()
        {
            World.AddConstraint(worldPointConstraint[0]);
            World.AddConstraint(worldPointConstraint[1]);
            World.AddConstraint(distance);
        }

        /// <summary>
        /// Removes the internal constraints of this joint from the world class.
        /// </summary>
        public override void Deactivate()
        {
            World.RemoveConstraint(worldPointConstraint[0]);
            World.RemoveConstraint(worldPointConstraint[1]);
            World.RemoveConstraint(distance);
        }
    }
}
