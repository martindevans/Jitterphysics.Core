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

#nullable enable

using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using Jitter.Collision;
using Jitter.Dynamics.Constraints;

namespace Jitter.Dynamics
{
    public enum RigidBodyIndex
    {
        RigidBody1, RigidBody2
    }

    /// <summary>
    /// The RigidBody class.
    /// </summary>
    public sealed class RigidBody
        : IDebugDrawable, IEquatable<RigidBody>, IComparable<RigidBody>
    {
        [Flags]
        public enum DampingType
        {
            None = 0,
            Angular = 1,
            Linear = 2
        }

        private JMatrix inertia;
        private JMatrix invInertia;

        internal JMatrix invInertiaWorld;
        internal JMatrix orientation;
        internal JMatrix invOrientation;
        internal Vector3 position;
        internal Vector3 linearVelocity;
        internal Vector3 angularVelocity;

        internal JBBox boundingBox;

        internal float inactiveTime;

        private bool _isActive = true;
        private bool _isStatic;
        private bool _affectedByGravity = true;

        internal CollisionIsland? island;
        internal float inverseMass;

        internal Vector3 AccumulatedForce;
        internal Vector3 AccumulatedTorque;

        private ShapeUpdatedHandler? updatedHandler;

        internal readonly List<RigidBody> connections = new();
        internal readonly HashSet<Arbiter> arbiters = new();
        internal readonly HashSet<Constraint> constraints = new();

        internal int marker = 0;

        private bool _isParticle;

        /// <summary>
        /// If true, the body as no angular movement.
        /// </summary>
        public bool IsParticle
        { 
            get => _isParticle;
            set
            {
                if (_isParticle == value)
                    return;

                if (_isParticle && !value)
                {
                    updatedHandler = ShapeUpdated;
                    Shape.ShapeUpdated += updatedHandler;
                    SetMassProperties();
                    _isParticle = false;
                }
                else if (!_isParticle && value)
                {
                    inertia = default;
                    invInertia = invInertiaWorld = default;
                    invOrientation = orientation = JMatrix.Identity;
                    inverseMass = 1.0f;

                    Shape.ShapeUpdated -= updatedHandler;

                    _isParticle = true;
                }

                Update();
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="shape"></param>
        public RigidBody(BaseShape shape)
            : this(shape, new())
        {
        }

        /// <summary>
        /// Initializes a new instance of the RigidBody class.
        /// </summary>
        /// <param name="shape">The shape of the body.</param>
        /// <param name="material"></param>
        /// <param name="isParticle">If set to true the body doesn't rotate. 
        /// Also contacts are only solved for the linear motion part.</param>
        public RigidBody(BaseShape shape, Material material, bool isParticle = false)
        {
            instance = Interlocked.Increment(ref instanceCount);

            // Setting shape twice. The property does some important setups (hooking events) so must be called.
            // However the property cannot handle a null shape. So first set the shape and then set the shape
            // for real.
            _shape = shape;
            Shape = shape;

            orientation = JMatrix.Identity;

            _isParticle = isParticle;
            if (!isParticle)
            {
                updatedHandler = ShapeUpdated;
                Shape.ShapeUpdated += updatedHandler;
                SetMassProperties();
            }
            else
            {
                inertia = default;
                invInertia = invInertiaWorld = default;
                invOrientation = orientation = JMatrix.Identity;
                inverseMass = 1.0f;
            }

            Material = material;

            AllowDeactivation = true;
            EnableSpeculativeContacts = false;

            Update();
        }

        /// <summary>
        /// Calculates a hashcode for this RigidBody.
        /// The hashcode should be unique as possible
        /// for every body.
        /// </summary>
        /// <returns>The hashcode.</returns>
        public override int GetHashCode()
        {
            return HashCode.Combine(instance, 1234);
        }

        public IReadOnlyCollection<Arbiter> Arbiters => arbiters;
        public IReadOnlyCollection<Constraint> Constraints => constraints;

        /// <summary>
        /// If set to false the body will never be deactived by the
        /// world.
        /// </summary>
        public bool AllowDeactivation { get; set; }

        public bool EnableSpeculativeContacts { get; set; }

        /// <summary>
        /// The axis aligned bounding box of the body.
        /// </summary>
        public JBBox BoundingBox => boundingBox;


        private static int instanceCount;
        private readonly int instance;

        /// <summary>
        /// Gets the current collision island the body is in.
        /// </summary>
        public CollisionIsland? CollisionIsland => island;

        /// <summary>
        /// If set to false the velocity is set to zero,
        /// the body gets immediately freezed.
        /// </summary>
        public bool IsActive
        {
            get => _isActive;
            set
            {
                if (!_isActive && value)
                {
                    // if inactive and should be active
                    inactiveTime = 0.0f;
                }
                else if (_isActive && !value)
                {
                    // if active and should be inactive
                    inactiveTime = float.PositiveInfinity;
                    angularVelocity = default;
                    linearVelocity = default;
                }

                _isActive = value;
            }
        }

        /// <summary>
        /// Applies an impulse on the center of the body. Changing
        /// linear velocity.
        /// </summary>
        /// <param name="impulse">Impulse direction and magnitude.</param>
        public void ApplyImpulse(Vector3 impulse)
        {
            if (_isStatic)
                throw new InvalidOperationException("Can't apply an impulse to a static body.");

            var temp = impulse * inverseMass;
            linearVelocity += temp;
        }

        /// <summary>
        /// Applies an impulse on the specific position. Changing linear
        /// and angular velocity.
        /// </summary>
        /// <param name="impulse">Impulse direction and magnitude.</param>
        /// <param name="relativePosition">The position where the impulse gets applied
        /// in Body coordinate frame.</param>
        public void ApplyImpulse(Vector3 impulse, Vector3 relativePosition)
        {
            if (_isStatic)
                throw new InvalidOperationException("Can't apply an impulse to a static body.");

            var temp = impulse * inverseMass;
            linearVelocity += temp;

            temp = Vector3.Cross(relativePosition, impulse);
            temp = JVectorExtensions.Transform(temp, invInertiaWorld);
            angularVelocity += temp;
        }

        /// <summary>
        /// Adds a force to the center of the body. The force gets applied
        /// the next time <see cref="World.Step(float)"/> is called. The 'impact'
        /// of the force depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="force">The force to add next <see cref="World.Step(float)"/>.</param>
        public void AddForce(Vector3 force)
        {
            AccumulatedForce = force + AccumulatedForce;
        }

        /// <summary>
        /// Adds a force to the center of the body. The force gets applied
        /// the next time <see cref="World.Step(float)"/> is called. The 'impact'
        /// of the force depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="force">The force to add next <see cref="World.Step(float)"/>.</param>
        /// <param name="pos">The position where the force is applied.</param>
        public void AddForce(Vector3 force, Vector3 pos)
        {
            AccumulatedForce += force;
            pos -= position;
            pos = Vector3.Cross(pos, force);
            AccumulatedTorque = pos + AccumulatedTorque;
        }

        /// <summary>
        /// Returns the torque which acts this timestep on the body.
        /// </summary>
        public Vector3 Torque => AccumulatedTorque;

        /// <summary>
        /// Returns the force which acts this timestep on the body.
        /// </summary>
        public Vector3 Force { get => AccumulatedForce;
            set => AccumulatedForce = value;
        }

        /// <summary>
        /// Adds torque to the body. The torque gets applied
        /// the next time <see cref="World.Step(float)"/> is called. The 'impact'
        /// of the torque depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="torque">The torque to add next <see cref="World.Step"/>.</param>
        public void AddTorque(Vector3 torque)
        {
            AccumulatedTorque = torque + AccumulatedTorque;
        }

        protected bool useShapeMassProperties = true;

        /// <summary>
        /// By calling this method the shape inertia and mass is used.
        /// </summary>
        public void SetMassProperties()
        {
            inertia = Shape.inertia;
            invInertia = inertia.Inverse();
            inverseMass = 1.0f / Shape.mass;
            useShapeMassProperties = true;
        }

        /// <summary>
        /// The engine used the given values for inertia and mass and ignores
        /// the shape mass properties.
        /// </summary>
        /// <param name="inertia">The inertia/inverse inertia of the untransformed object.</param>
        /// <param name="mass">The mass/inverse mass of the object.</param>
        /// <param name="setAsInverseValues">Sets the InverseInertia and the InverseMass
        /// to this values.</param>
        public void SetMassProperties(JMatrix inertia, float mass, bool setAsInverseValues)
        {
            if (setAsInverseValues)
            {
                if (!_isParticle)
                {
                    invInertia = inertia;
                    this.inertia = inertia.Inverse();
                    this.inertia = inertia.Inverse();
                }
                inverseMass = mass;
            }
            else
            {
                if (!_isParticle)
                {
                    this.inertia = inertia;
                    invInertia = inertia.Inverse();
                }
                inverseMass = 1.0f / mass;
            }

            useShapeMassProperties = false;
            Update();
        }

        private void ShapeUpdated()
        {
            if (useShapeMassProperties) SetMassProperties();
            Update();
            UpdateHullData();
        }

        /// <summary>
        /// The shape the body is using.
        /// </summary>
        public BaseShape Shape 
        {
            get => _shape;
            set 
            {
                // deregister update event
                _shape.ShapeUpdated -= updatedHandler;

                // register new event
                _shape = value; 
                _shape.ShapeUpdated += ShapeUpdated; 
            } 
        }

        private BaseShape _shape;

        public DampingType Damping { get; set; } = DampingType.Angular | DampingType.Linear;

        public Material Material { get; set; }

        /// <summary>
        /// The inertia currently used for this body.
        /// </summary>
        public JMatrix Inertia => inertia;

        /// <summary>
        /// The inverse inertia currently used for this body.
        /// </summary>
        public JMatrix InverseInertia => invInertia;

        /// <summary>
        /// The velocity of the body.
        /// </summary>
        public Vector3 LinearVelocity
        {
            get => linearVelocity;
            set 
            { 
                if (_isStatic) 
                    throw new InvalidOperationException("Can't set a velocity to a static body.");
                linearVelocity = value;
            }
        }

        // TODO: check here is static!
        /// <summary>
        /// The angular velocity of the body.
        /// </summary>
        public Vector3 AngularVelocity
        {
            get => angularVelocity;
            set
            {
                if (_isStatic)
                    throw new InvalidOperationException("Can't set a velocity to a static body.");
                angularVelocity = value;
            }
        }

        /// <summary>
        /// The current position of the body.
        /// </summary>
        public Vector3 Position
        {
            get => position;
            set { position = value ; Update(); }
        }

        /// <summary>
        /// The current oriention of the body.
        /// </summary>
        public JMatrix Orientation
        {
            get => orientation;
            set { orientation = value; Update(); }
        }

        /// <summary>
        /// If set to true the body can't be moved.
        /// </summary>
        public bool IsStatic
        {
            get => _isStatic;
            set
            {
                if (value && !_isStatic)
                {
                    island?.islandManager.MakeBodyStatic(this);

                    angularVelocity = default;
                    linearVelocity = default;
                }
                _isStatic = value;
            }
        }

        public bool AffectedByGravity
        {
            get => _affectedByGravity;
            set => _affectedByGravity = value;
        }

        /// <summary>
        /// The inverse inertia tensor in world space.
        /// </summary>
        public JMatrix InverseInertiaWorld => invInertiaWorld;

        /// <summary>
        /// Setting the mass automatically scales the inertia.
        /// To set the mass indepedently from the mass use SetMassProperties.
        /// </summary>
        public float Mass
        {
            get => 1.0f / inverseMass;
            set 
            {
                if (value <= 0.0f) throw new ArgumentException("Mass can't be less or equal zero.");

                // scale inertia
                if (!_isParticle)
                {
                    JMatrix.Multiply(ref Shape.inertia, value / Shape.mass, out inertia);
                    invInertia = inertia.Inverse();
                }

                inverseMass = 1.0f / value;
            }
        }


        internal Vector3 sweptDirection;

        public void SweptExpandBoundingBox(float timestep)
        {
            sweptDirection = linearVelocity * timestep;

            if (sweptDirection.X < 0.0f)
            {
                boundingBox.Min.X += sweptDirection.X;
            }
            else
            {
                boundingBox.Max.X += sweptDirection.X;
            }

            if (sweptDirection.Y < 0.0f)
            {
                boundingBox.Min.Y += sweptDirection.Y;
            }
            else
            {
                boundingBox.Max.Y += sweptDirection.Y;
            }

            if (sweptDirection.Z < 0.0f)
            {
                boundingBox.Min.Z += sweptDirection.Z;
            }
            else
            {
                boundingBox.Max.Z += sweptDirection.Z;
            }
        }

        /// <summary>
        /// Recalculates the axis aligned bounding box and the inertia
        /// values in world space.
        /// </summary>
        public void Update()
        {
            if (_isParticle)
            {
                inertia = default;
                invInertia = invInertiaWorld = default;
                invOrientation = orientation = JMatrix.Identity;
                boundingBox = _shape.boundingBox;
                boundingBox.Min += position;
                boundingBox.Max += position;

                angularVelocity = default;
            }
            else
            {
                // Given: Orientation, Inertia
                JMatrix.Transpose(ref orientation, out invOrientation);
                boundingBox = Shape.GetBoundingBox(orientation);
                boundingBox.Min += position;
                boundingBox.Max += position;


                if (!_isStatic)
                {
                    JMatrix.Multiply(ref invOrientation, ref invInertia, out invInertiaWorld);
                    JMatrix.Multiply(ref invInertiaWorld, ref orientation, out invInertiaWorld);
                }
            }
        }

        /// <inheritdoc />
        public bool Equals(RigidBody? other)
        {
            return other?.instance == instance;
        }

        /// <inheritdoc/>
        public int CompareTo(RigidBody? other)
        {
            return other?.instance.CompareTo(instance) ?? 1;
        }

        public bool IsStaticOrInactive => !_isActive || _isStatic;

        private bool enableDebugDraw;
        public bool EnableDebugDraw
        {
            get => enableDebugDraw;
            set
            {
                enableDebugDraw = value;
                UpdateHullData();
            }
        }

        private readonly List<Vector3> hullPoints = new();

        private void UpdateHullData()
        {
            hullPoints.Clear();

            if (enableDebugDraw)
                _shape.MakeHull(hullPoints, 3);
        }


        /// <inheritdoc />
        public void DebugDraw(IDebugDrawer drawer)
        {
            for(var i = 0;i<hullPoints.Count;i+=3)
            {
                var pos1 = hullPoints[i + 0];
                var pos2 = hullPoints[i + 1];
                var pos3 = hullPoints[i + 2];

                pos1 = JVectorExtensions.Transform(pos1, orientation);
                pos1 += position;

                pos2 = JVectorExtensions.Transform(pos2, orientation);
                pos2 += position;

                pos3 = JVectorExtensions.Transform(pos3, orientation);
                pos3 += position;

                drawer.DrawTriangle(pos1, pos2, pos3);
            }
        }
    }
}
