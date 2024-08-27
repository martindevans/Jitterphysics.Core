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
using Jitter.LinearMath;

namespace Jitter.Dynamics
{
    public class ContactSettings
    {
        public enum MaterialCoefficientMixingType { TakeMaximum, TakeMinimum, UseAverage }

        internal float maximumBias = 10.0f;
        internal float bias = 0.25f;
        internal float minVelocity = 0.001f;
        internal float allowedPenetration = 0.01f;
        internal float breakThreshold = 0.01f;

        internal MaterialCoefficientMixingType materialMode = MaterialCoefficientMixingType.UseAverage;

        public float MaximumBias { get => maximumBias;
            set => maximumBias = value;
        }

        public float BiasFactor { get => bias;
            set => bias = value;
        }

        public float MinimumVelocity { get => minVelocity;
            set => minVelocity = value;
        }

        public float AllowedPenetration { get => allowedPenetration;
            set => allowedPenetration = value;
        }

        public float BreakThreshold { get => breakThreshold;
            set => breakThreshold = value;
        }

        public MaterialCoefficientMixingType MaterialCoefficientMixing { get => materialMode;
            set => materialMode = value;
        }
    }


    /// <summary>
    /// </summary>
    public class Contact : IConstraint
    {
        private ContactSettings settings;

        internal RigidBody body1, body2;

        internal JVector normal, tangent;

        internal JVector realRelPos1, realRelPos2;
        internal JVector relativePos1, relativePos2;
        internal JVector p1, p2;

        internal float accumulatedNormalImpulse;
        internal float accumulatedTangentImpulse;

        internal float penetration;
        internal float initialPen;

        private float staticFriction, dynamicFriction, restitution;
        private float friction;

        private float massNormal, massTangent;
        private float restitutionBias;

        private bool newContact;

        private bool treatBody1AsStatic;
        private bool treatBody2AsStatic;


        bool body1IsMassPoint; bool body2IsMassPoint;

        float lostSpeculativeBounce;
        float speculativeVelocity;

        /// <summary>
        /// A contact resource pool.
        /// </summary>
        public static readonly ResourcePool<Contact> Pool =
            new ResourcePool<Contact>();

        private float lastTimeStep = float.PositiveInfinity;

        public float Restitution
        {
            get => restitution;
            set => restitution = value;
        }

        public float StaticFriction
        {
            get => staticFriction;
            set => staticFriction = value;
        }

        public float DynamicFriction
        {
            get => dynamicFriction;
            set => dynamicFriction = value;
        }

        /// <summary>
        /// The first body involved in the contact.
        /// </summary>
        public RigidBody Body1 => body1;

        /// <summary>
        /// The second body involved in the contact.
        /// </summary>
        public RigidBody Body2 => body2;

        /// <summary>
        /// The penetration of the contact.
        /// </summary>
        public float Penetration => penetration;

        /// <summary>
        /// The collision position in world space of body1.
        /// </summary>
        public JVector Position1 => p1;

        /// <summary>
        /// The collision position in world space of body2.
        /// </summary>
        public JVector Position2 => p2;

        /// <summary>
        /// The contact tangent.
        /// </summary>
        public JVector Tangent => tangent;

        /// <summary>
        /// The contact normal.
        /// </summary>
        public JVector Normal => normal;

        /// <summary>
        /// Calculates relative velocity of body contact points on the bodies.
        /// </summary>
        /// <param name="relVel">The relative velocity of body contact points on the bodies.</param>
        public JVector CalculateRelativeVelocity()
        {
            float x, y, z;

            x = body2.angularVelocity.Y * relativePos2.Z - body2.angularVelocity.Z * relativePos2.Y + body2.linearVelocity.X;
            y = body2.angularVelocity.Z * relativePos2.X - body2.angularVelocity.X * relativePos2.Z + body2.linearVelocity.Y;
            z = body2.angularVelocity.X * relativePos2.Y - body2.angularVelocity.Y * relativePos2.X + body2.linearVelocity.Z;

            JVector relVel;
            relVel.X = x - body1.angularVelocity.Y * relativePos1.Z + body1.angularVelocity.Z * relativePos1.Y - body1.linearVelocity.X;
            relVel.Y = y - body1.angularVelocity.Z * relativePos1.X + body1.angularVelocity.X * relativePos1.Z - body1.linearVelocity.Y;
            relVel.Z = z - body1.angularVelocity.X * relativePos1.Y + body1.angularVelocity.Y * relativePos1.X - body1.linearVelocity.Z;

            return relVel;
        }

        /// <summary>
        /// Solves the contact iteratively.
        /// </summary>
        public void Iterate()
        {
            //body1.linearVelocity = JVector.Zero;
            //body2.linearVelocity = JVector.Zero;
            //return;

            if (treatBody1AsStatic && treatBody2AsStatic) return;

            float dvx, dvy, dvz;

            dvx = body2.linearVelocity.X - body1.linearVelocity.X;
            dvy = body2.linearVelocity.Y - body1.linearVelocity.Y;
            dvz = body2.linearVelocity.Z - body1.linearVelocity.Z;

            if (!body1IsMassPoint)
            {
                dvx = dvx - body1.angularVelocity.Y * relativePos1.Z + body1.angularVelocity.Z * relativePos1.Y;
                dvy = dvy - body1.angularVelocity.Z * relativePos1.X + body1.angularVelocity.X * relativePos1.Z;
                dvz = dvz - body1.angularVelocity.X * relativePos1.Y + body1.angularVelocity.Y * relativePos1.X;
            }

            if (!body2IsMassPoint)
            {
                dvx = dvx + body2.angularVelocity.Y * relativePos2.Z - body2.angularVelocity.Z * relativePos2.Y;
                dvy = dvy + body2.angularVelocity.Z * relativePos2.X - body2.angularVelocity.X * relativePos2.Z;
                dvz = dvz + body2.angularVelocity.X * relativePos2.Y - body2.angularVelocity.Y * relativePos2.X;
            }

            // this gets us some performance
            if (dvx * dvx + dvy * dvy + dvz * dvz < settings.minVelocity * settings.minVelocity)
            { return; }

            var vn = normal.X * dvx + normal.Y * dvy + normal.Z * dvz;
            var normalImpulse = massNormal * (-vn + restitutionBias + speculativeVelocity);

            var oldNormalImpulse = accumulatedNormalImpulse;
            accumulatedNormalImpulse = oldNormalImpulse + normalImpulse;
            if (accumulatedNormalImpulse < 0.0f) accumulatedNormalImpulse = 0.0f;
            normalImpulse = accumulatedNormalImpulse - oldNormalImpulse;

            var vt = dvx * tangent.X + dvy * tangent.Y + dvz * tangent.Z;
            var maxTangentImpulse = friction * accumulatedNormalImpulse;
            var tangentImpulse = massTangent * -vt;

            var oldTangentImpulse = accumulatedTangentImpulse;
            accumulatedTangentImpulse = oldTangentImpulse + tangentImpulse;
            if (accumulatedTangentImpulse < -maxTangentImpulse) accumulatedTangentImpulse = -maxTangentImpulse;
            else if (accumulatedTangentImpulse > maxTangentImpulse) accumulatedTangentImpulse = maxTangentImpulse;

            tangentImpulse = accumulatedTangentImpulse - oldTangentImpulse;

            // Apply contact impulse
            JVector impulse;
            impulse.X = normal.X * normalImpulse + tangent.X * tangentImpulse;
            impulse.Y = normal.Y * normalImpulse + tangent.Y * tangentImpulse;
            impulse.Z = normal.Z * normalImpulse + tangent.Z * tangentImpulse;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= impulse.X * body1.inverseMass;
                body1.linearVelocity.Y -= impulse.Y * body1.inverseMass;
                body1.linearVelocity.Z -= impulse.Z * body1.inverseMass;

                if (!body1IsMassPoint)
                {
                    float num0, num1, num2;
                    num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
                    num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
                    num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;

                    var num3 =
                        num0 * body1.invInertiaWorld.M11 +
                        num1 * body1.invInertiaWorld.M21 +
                        num2 * body1.invInertiaWorld.M31;
                    var num4 =
                        num0 * body1.invInertiaWorld.M12 +
                        num1 * body1.invInertiaWorld.M22 +
                        num2 * body1.invInertiaWorld.M32;
                    var num5 =
                        num0 * body1.invInertiaWorld.M13 +
                        num1 * body1.invInertiaWorld.M23 +
                        num2 * body1.invInertiaWorld.M33;

                    body1.angularVelocity.X -= num3;
                    body1.angularVelocity.Y -= num4;
                    body1.angularVelocity.Z -= num5;
                }
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += impulse.X * body2.inverseMass;
                body2.linearVelocity.Y += impulse.Y * body2.inverseMass;
                body2.linearVelocity.Z += impulse.Z * body2.inverseMass;

                if (!body2IsMassPoint)
                {

                    float num0, num1, num2;
                    num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
                    num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
                    num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;

                    var num3 =
                        num0 * body2.invInertiaWorld.M11 +
                        num1 * body2.invInertiaWorld.M21 +
                        num2 * body2.invInertiaWorld.M31;
                    var num4 =
                        num0 * body2.invInertiaWorld.M12 +
                        num1 * body2.invInertiaWorld.M22 +
                        num2 * body2.invInertiaWorld.M32;
                    var num5 =
                        num0 * body2.invInertiaWorld.M13 +
                        num1 * body2.invInertiaWorld.M23 +
                        num2 * body2.invInertiaWorld.M33;

                    body2.angularVelocity.X += num3;
                    body2.angularVelocity.Y += num4;
                    body2.angularVelocity.Z += num5;

                }
            }

        }

        public float AppliedNormalImpulse => accumulatedNormalImpulse;
        public float AppliedTangentImpulse => accumulatedTangentImpulse;

        /// <summary>
        /// The points in wolrd space gets recalculated by transforming the
        /// local coordinates. Also new penetration depth is estimated.
        /// </summary>
        public void UpdatePosition()
        {
            if (body1IsMassPoint)
            {
                p1 = JVector.Add(realRelPos1, body1.position);
            }
            else
            {
                JVector.Transform(ref realRelPos1, ref body1.orientation, out p1);
                p1 = JVector.Add(p1, body1.position);
            }

            if (body2IsMassPoint)
            {
                p2 = JVector.Add(realRelPos2, body2.position);
            }
            else
            {
                JVector.Transform(ref realRelPos2, ref body2.orientation, out p2);
                p2 = JVector.Add(p2, body2.position);
            }


            var dist = JVector.Subtract(p1, p2);
            penetration = JVector.Dot(dist, normal);
        }

        /// <summary>
        /// An impulse is applied an both contact points.
        /// </summary>
        /// <param name="impulse">The impulse to apply.</param>
        public void ApplyImpulse(ref JVector impulse)
        {
            //JVector temp;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= impulse.X * body1.inverseMass;
                body1.linearVelocity.Y -= impulse.Y * body1.inverseMass;
                body1.linearVelocity.Z -= impulse.Z * body1.inverseMass;

                float num0, num1, num2;
                num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
                num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
                num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;

                var num3 =
                    num0 * body1.invInertiaWorld.M11 +
                    num1 * body1.invInertiaWorld.M21 +
                    num2 * body1.invInertiaWorld.M31;
                var num4 =
                    num0 * body1.invInertiaWorld.M12 +
                    num1 * body1.invInertiaWorld.M22 +
                    num2 * body1.invInertiaWorld.M32;
                var num5 =
                    num0 * body1.invInertiaWorld.M13 +
                    num1 * body1.invInertiaWorld.M23 +
                    num2 * body1.invInertiaWorld.M33;

                body1.angularVelocity.X -= num3;
                body1.angularVelocity.Y -= num4;
                body1.angularVelocity.Z -= num5;
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += impulse.X * body2.inverseMass;
                body2.linearVelocity.Y += impulse.Y * body2.inverseMass;
                body2.linearVelocity.Z += impulse.Z * body2.inverseMass;

                float num0, num1, num2;
                num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
                num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
                num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;

                var num3 =
                    num0 * body2.invInertiaWorld.M11 +
                    num1 * body2.invInertiaWorld.M21 +
                    num2 * body2.invInertiaWorld.M31;
                var num4 =
                    num0 * body2.invInertiaWorld.M12 +
                    num1 * body2.invInertiaWorld.M22 +
                    num2 * body2.invInertiaWorld.M32;
                var num5 =
                    num0 * body2.invInertiaWorld.M13 +
                    num1 * body2.invInertiaWorld.M23 +
                    num2 * body2.invInertiaWorld.M33;

                body2.angularVelocity.X += num3;
                body2.angularVelocity.Y += num4;
                body2.angularVelocity.Z += num5;
            }
        }

        public void ApplyImpulse(JVector impulse)
        {
            //JVector temp;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= impulse.X * body1.inverseMass;
                body1.linearVelocity.Y -= impulse.Y * body1.inverseMass;
                body1.linearVelocity.Z -= impulse.Z * body1.inverseMass;

                float num0, num1, num2;
                num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
                num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
                num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;

                var num3 =
                    num0 * body1.invInertiaWorld.M11 +
                    num1 * body1.invInertiaWorld.M21 +
                    num2 * body1.invInertiaWorld.M31;
                var num4 =
                    num0 * body1.invInertiaWorld.M12 +
                    num1 * body1.invInertiaWorld.M22 +
                    num2 * body1.invInertiaWorld.M32;
                var num5 =
                    num0 * body1.invInertiaWorld.M13 +
                    num1 * body1.invInertiaWorld.M23 +
                    num2 * body1.invInertiaWorld.M33;

                body1.angularVelocity.X -= num3;
                body1.angularVelocity.Y -= num4;
                body1.angularVelocity.Z -= num5;
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += impulse.X * body2.inverseMass;
                body2.linearVelocity.Y += impulse.Y * body2.inverseMass;
                body2.linearVelocity.Z += impulse.Z * body2.inverseMass;

                float num0, num1, num2;
                num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
                num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
                num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;

                var num3 =
                    num0 * body2.invInertiaWorld.M11 +
                    num1 * body2.invInertiaWorld.M21 +
                    num2 * body2.invInertiaWorld.M31;
                var num4 =
                    num0 * body2.invInertiaWorld.M12 +
                    num1 * body2.invInertiaWorld.M22 +
                    num2 * body2.invInertiaWorld.M32;
                var num5 =
                    num0 * body2.invInertiaWorld.M13 +
                    num1 * body2.invInertiaWorld.M23 +
                    num2 * body2.invInertiaWorld.M33;

                body2.angularVelocity.X += num3;
                body2.angularVelocity.Y += num4;
                body2.angularVelocity.Z += num5;
            }
        }

        /// <summary>
        /// PrepareForIteration has to be called before <see cref="Iterate"/>.
        /// </summary>
        /// <param name="timestep">The timestep of the simulation.</param>
        public void PrepareForIteration(float timestep)
        {
            float dvx, dvy, dvz;

            dvx = body2.angularVelocity.Y * relativePos2.Z - body2.angularVelocity.Z * relativePos2.Y + body2.linearVelocity.X;
            dvy = body2.angularVelocity.Z * relativePos2.X - body2.angularVelocity.X * relativePos2.Z + body2.linearVelocity.Y;
            dvz = body2.angularVelocity.X * relativePos2.Y - body2.angularVelocity.Y * relativePos2.X + body2.linearVelocity.Z;

            dvx = dvx - body1.angularVelocity.Y * relativePos1.Z + body1.angularVelocity.Z * relativePos1.Y - body1.linearVelocity.X;
            dvy = dvy - body1.angularVelocity.Z * relativePos1.X + body1.angularVelocity.X * relativePos1.Z - body1.linearVelocity.Y;
            dvz = dvz - body1.angularVelocity.X * relativePos1.Y + body1.angularVelocity.Y * relativePos1.X - body1.linearVelocity.Z;

            var kNormal = 0.0f;

            var rantra = JVector.Zero;
            if (!treatBody1AsStatic)
            {
                kNormal += body1.inverseMass;

                if (!body1IsMassPoint)
                {

                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    rantra.X = relativePos1.Y * normal.Z - relativePos1.Z * normal.Y;
                    rantra.Y = relativePos1.Z * normal.X - relativePos1.X * normal.Z;
                    rantra.Z = relativePos1.X * normal.Y - relativePos1.Y * normal.X;

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    var num0 = rantra.X * body1.invInertiaWorld.M11 + rantra.Y * body1.invInertiaWorld.M21 + rantra.Z * body1.invInertiaWorld.M31;
                    var num1 = rantra.X * body1.invInertiaWorld.M12 + rantra.Y * body1.invInertiaWorld.M22 + rantra.Z * body1.invInertiaWorld.M32;
                    var num2 = rantra.X * body1.invInertiaWorld.M13 + rantra.Y * body1.invInertiaWorld.M23 + rantra.Z * body1.invInertiaWorld.M33;

                    rantra.X = num0; rantra.Y = num1; rantra.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    num0 = rantra.Y * relativePos1.Z - rantra.Z * relativePos1.Y;
                    num1 = rantra.Z * relativePos1.X - rantra.X * relativePos1.Z;
                    num2 = rantra.X * relativePos1.Y - rantra.Y * relativePos1.X;

                    rantra.X = num0; rantra.Y = num1; rantra.Z = num2;
                }
            }

            var rbntrb = JVector.Zero;
            if (!treatBody2AsStatic)
            {
                kNormal += body2.inverseMass;

                if (!body2IsMassPoint)
                {

                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    rbntrb.X = relativePos2.Y * normal.Z - relativePos2.Z * normal.Y;
                    rbntrb.Y = relativePos2.Z * normal.X - relativePos2.X * normal.Z;
                    rbntrb.Z = relativePos2.X * normal.Y - relativePos2.Y * normal.X;

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    var num0 = rbntrb.X * body2.invInertiaWorld.M11 + rbntrb.Y * body2.invInertiaWorld.M21 + rbntrb.Z * body2.invInertiaWorld.M31;
                    var num1 = rbntrb.X * body2.invInertiaWorld.M12 + rbntrb.Y * body2.invInertiaWorld.M22 + rbntrb.Z * body2.invInertiaWorld.M32;
                    var num2 = rbntrb.X * body2.invInertiaWorld.M13 + rbntrb.Y * body2.invInertiaWorld.M23 + rbntrb.Z * body2.invInertiaWorld.M33;

                    rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    num0 = rbntrb.Y * relativePos2.Z - rbntrb.Z * relativePos2.Y;
                    num1 = rbntrb.Z * relativePos2.X - rbntrb.X * relativePos2.Z;
                    num2 = rbntrb.X * relativePos2.Y - rbntrb.Y * relativePos2.X;

                    rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;
                }
            }

            if (!treatBody1AsStatic) kNormal += rantra.X * normal.X + rantra.Y * normal.Y + rantra.Z * normal.Z;
            if (!treatBody2AsStatic) kNormal += rbntrb.X * normal.X + rbntrb.Y * normal.Y + rbntrb.Z * normal.Z;

            massNormal = 1.0f / kNormal;

            var num = dvx * normal.X + dvy * normal.Y + dvz * normal.Z;

            tangent.X = dvx - normal.X * num;
            tangent.Y = dvy - normal.Y * num;
            tangent.Z = dvz - normal.Z * num;

            num = tangent.X * tangent.X + tangent.Y * tangent.Y + tangent.Z * tangent.Z;

            if (num != 0.0f)
            {
                num = (float)Math.Sqrt(num);
                tangent.X /= num;
                tangent.Y /= num;
                tangent.Z /= num;
            }

            var kTangent = 0.0f;

            if (treatBody1AsStatic)
                rantra = JVector.Zero;
            else
            {
                kTangent += body1.inverseMass;
  
                if (!body1IsMassPoint)
                {
                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    rantra.X = relativePos1.Y * tangent.Z - relativePos1.Z * tangent.Y;
                    rantra.Y = relativePos1.Z * tangent.X - relativePos1.X * tangent.Z;
                    rantra.Z = relativePos1.X * tangent.Y - relativePos1.Y * tangent.X;

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    var num0 = rantra.X * body1.invInertiaWorld.M11 + rantra.Y * body1.invInertiaWorld.M21 + rantra.Z * body1.invInertiaWorld.M31;
                    var num1 = rantra.X * body1.invInertiaWorld.M12 + rantra.Y * body1.invInertiaWorld.M22 + rantra.Z * body1.invInertiaWorld.M32;
                    var num2 = rantra.X * body1.invInertiaWorld.M13 + rantra.Y * body1.invInertiaWorld.M23 + rantra.Z * body1.invInertiaWorld.M33;

                    rantra.X = num0; rantra.Y = num1; rantra.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    num0 = rantra.Y * relativePos1.Z - rantra.Z * relativePos1.Y;
                    num1 = rantra.Z * relativePos1.X - rantra.X * relativePos1.Z;
                    num2 = rantra.X * relativePos1.Y - rantra.Y * relativePos1.X;

                    rantra.X = num0; rantra.Y = num1; rantra.Z = num2;
                }

            }

            if (treatBody2AsStatic)
                rbntrb = JVector.Zero;
            else
            {
                kTangent += body2.inverseMass;

                if (!body2IsMassPoint)
                {
                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    rbntrb.X = relativePos2.Y * tangent.Z - relativePos2.Z * tangent.Y;
                    rbntrb.Y = relativePos2.Z * tangent.X - relativePos2.X * tangent.Z;
                    rbntrb.Z = relativePos2.X * tangent.Y - relativePos2.Y * tangent.X;

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    var num0 = rbntrb.X * body2.invInertiaWorld.M11 + rbntrb.Y * body2.invInertiaWorld.M21 + rbntrb.Z * body2.invInertiaWorld.M31;
                    var num1 = rbntrb.X * body2.invInertiaWorld.M12 + rbntrb.Y * body2.invInertiaWorld.M22 + rbntrb.Z * body2.invInertiaWorld.M32;
                    var num2 = rbntrb.X * body2.invInertiaWorld.M13 + rbntrb.Y * body2.invInertiaWorld.M23 + rbntrb.Z * body2.invInertiaWorld.M33;

                    rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    num0 = rbntrb.Y * relativePos2.Z - rbntrb.Z * relativePos2.Y;
                    num1 = rbntrb.Z * relativePos2.X - rbntrb.X * relativePos2.Z;
                    num2 = rbntrb.X * relativePos2.Y - rbntrb.Y * relativePos2.X;

                    rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;
                }
            }

            if (!treatBody1AsStatic) kTangent += JVector.Dot(rantra, tangent);
            if (!treatBody2AsStatic) kTangent += JVector.Dot(rbntrb, tangent);
            massTangent = 1.0f / kTangent;

            restitutionBias = lostSpeculativeBounce;

            speculativeVelocity = 0.0f;

            var relNormalVel = normal.X * dvx + normal.Y * dvy + normal.Z * dvz; //JVector.Dot(ref normal, ref dv);

            if (Penetration > settings.allowedPenetration)
            {
                restitutionBias = settings.bias * (1.0f / timestep) * Math.Max(0.0f, Penetration - settings.allowedPenetration);
                restitutionBias = Math.Clamp(restitutionBias, 0.0f, settings.maximumBias);
              //  body1IsMassPoint = body2IsMassPoint = false;
            }
      

            var timeStepRatio = timestep / lastTimeStep;
            accumulatedNormalImpulse *= timeStepRatio;
            accumulatedTangentImpulse *= timeStepRatio;

            {
                // Static/Dynamic friction
                var relTangentVel = -(tangent.X * dvx + tangent.Y * dvy + tangent.Z * dvz);
                var tangentImpulse = massTangent * relTangentVel;
                var maxTangentImpulse = -staticFriction * accumulatedNormalImpulse;

                if (tangentImpulse < maxTangentImpulse) friction = dynamicFriction;
                else friction = staticFriction;
            }

            JVector impulse;

            // Simultaneos solving and restitution is simply not possible
            // so fake it a bit by just applying restitution impulse when there
            // is a new contact.
            if (relNormalVel < -1.0f && newContact)
            {
                restitutionBias = Math.Max(-restitution * relNormalVel, restitutionBias);
            }

            // Speculative Contacts!
            // if the penetration is negative (which means the bodies are not already in contact, but they will
            // be in the future) we store the current bounce bias in the variable 'lostSpeculativeBounce'
            // and apply it the next frame, when the speculative contact was already solved.
            if (penetration < -settings.allowedPenetration)
            {
                speculativeVelocity = penetration / timestep;

                lostSpeculativeBounce = restitutionBias;
                restitutionBias = 0.0f;
            }
            else
            {
                lostSpeculativeBounce = 0.0f;
            }

            impulse.X = normal.X * accumulatedNormalImpulse + tangent.X * accumulatedTangentImpulse;
            impulse.Y = normal.Y * accumulatedNormalImpulse + tangent.Y * accumulatedTangentImpulse;
            impulse.Z = normal.Z * accumulatedNormalImpulse + tangent.Z * accumulatedTangentImpulse;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= impulse.X * body1.inverseMass;
                body1.linearVelocity.Y -= impulse.Y * body1.inverseMass;
                body1.linearVelocity.Z -= impulse.Z * body1.inverseMass;

                if (!body1IsMassPoint)
                {
                    float num0, num1, num2;
                    num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
                    num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
                    num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;

                    var num3 =
                        num0 * body1.invInertiaWorld.M11 +
                        num1 * body1.invInertiaWorld.M21 +
                        num2 * body1.invInertiaWorld.M31;
                    var num4 =
                        num0 * body1.invInertiaWorld.M12 +
                        num1 * body1.invInertiaWorld.M22 +
                        num2 * body1.invInertiaWorld.M32;
                    var num5 =
                        num0 * body1.invInertiaWorld.M13 +
                        num1 * body1.invInertiaWorld.M23 +
                        num2 * body1.invInertiaWorld.M33;

                    body1.angularVelocity.X -= num3;
                    body1.angularVelocity.Y -= num4;
                    body1.angularVelocity.Z -= num5;

                }
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += impulse.X * body2.inverseMass;
                body2.linearVelocity.Y += impulse.Y * body2.inverseMass;
                body2.linearVelocity.Z += impulse.Z * body2.inverseMass;

                if (!body2IsMassPoint)
                {

                    float num0, num1, num2;
                    num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
                    num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
                    num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;

                    var num3 =
                        num0 * body2.invInertiaWorld.M11 +
                        num1 * body2.invInertiaWorld.M21 +
                        num2 * body2.invInertiaWorld.M31;
                    var num4 =
                        num0 * body2.invInertiaWorld.M12 +
                        num1 * body2.invInertiaWorld.M22 +
                        num2 * body2.invInertiaWorld.M32;
                    var num5 =
                        num0 * body2.invInertiaWorld.M13 +
                        num1 * body2.invInertiaWorld.M23 +
                        num2 * body2.invInertiaWorld.M33;

                    body2.angularVelocity.X += num3;
                    body2.angularVelocity.Y += num4;
                    body2.angularVelocity.Z += num5;
                }
            }

            lastTimeStep = timestep;

            newContact = false;
        }

        public void TreatBodyAsStatic(RigidBodyIndex index)
        {
            if (index == RigidBodyIndex.RigidBody1) treatBody1AsStatic = true;
            else treatBody2AsStatic = true;
        }


        /// <summary>
        /// Initializes a contact.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <param name="point1">The collision point in worldspace</param>
        /// <param name="point2">The collision point in worldspace</param>
        /// <param name="n">The normal pointing to body2.</param>
        /// <param name="penetration">The estimated penetration depth.</param>
        public void Initialize(RigidBody body1, RigidBody body2, ref JVector point1, ref JVector point2, ref JVector n,
            float penetration, bool newContact, ContactSettings settings)
        {
            this.body1 = body1;  this.body2 = body2;
            normal = n; normal.Normalize();
            p1 = point1; p2 = point2;

            this.newContact = newContact;

            relativePos1 = JVector.Subtract(p1, body1.position);
            relativePos2 = JVector.Subtract(p2, body2.position);
            JVector.Transform(ref relativePos1, ref body1.invOrientation, out realRelPos1);
            JVector.Transform(ref relativePos2, ref body2.invOrientation, out realRelPos2);

            initialPen = penetration;
            this.penetration = penetration;

            body1IsMassPoint = body1.isParticle;
            body2IsMassPoint = body2.isParticle;

            // Material Properties
            if (newContact)
            {
                treatBody1AsStatic = body1.isStatic;
                treatBody2AsStatic = body2.isStatic;

                accumulatedNormalImpulse = 0.0f;
                accumulatedTangentImpulse = 0.0f;

                lostSpeculativeBounce = 0.0f;

                switch (settings.MaterialCoefficientMixing)
                {
                    case ContactSettings.MaterialCoefficientMixingType.TakeMaximum:
                        staticFriction = Math.Max(body1.material.StaticFriction, body2.material.StaticFriction);
                        dynamicFriction = Math.Max(body1.material.KineticFriction, body2.material.KineticFriction);
                        restitution = Math.Max(body1.material.Restitution, body2.material.Restitution);
                        break;
                    case ContactSettings.MaterialCoefficientMixingType.TakeMinimum:
                        staticFriction = Math.Min(body1.material.StaticFriction, body2.material.StaticFriction);
                        dynamicFriction = Math.Min(body1.material.KineticFriction, body2.material.KineticFriction);
                        restitution = Math.Min(body1.material.Restitution, body2.material.Restitution);
                        break;
                    case ContactSettings.MaterialCoefficientMixingType.UseAverage:
                        staticFriction = (body1.material.StaticFriction + body2.material.StaticFriction) / 2.0f;
                        dynamicFriction = (body1.material.KineticFriction + body2.material.KineticFriction) / 2.0f;
                        restitution = (body1.material.Restitution + body2.material.Restitution) / 2.0f;
                        break;
                }

            }

            this.settings = settings;
            


        }
    }
}
