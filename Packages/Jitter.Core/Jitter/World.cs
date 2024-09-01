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
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Numerics;
using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision;
using Jitter.Dynamics.Constraints;

namespace Jitter
{

    /// <summary>
    /// This class brings 'dynamics' and 'collisions' together. It handles
    /// all bodies and constraints.
    /// </summary>
    public sealed class World
    {
        /// <summary>
        /// Callback on every world step
        /// </summary>
        /// <param name="timestep"></param>
        public delegate void WorldStep(float timestep);

        /// <summary>
        /// Container for all events happening in a single world
        /// </summary>
        public class WorldEvents
        {
            // Post&Prestep
            public event WorldStep? PreStep;
            public event WorldStep? PostStep;

            // Add&Remove
            public event Action<RigidBody>? AddedRigidBody;
            public event Action<RigidBody>? RemovedRigidBody;
            public event Action<BaseConstraint>? AddedConstraint;
            public event Action<BaseConstraint>? RemovedConstraint;

            // Collision
            public event Action<RigidBody, RigidBody>? BodiesBeginCollide;
            public event Action<RigidBody, RigidBody>? BodiesEndCollide;
            public event Action<Contact>? ContactCreated;

            // Deactivation
            public event Action<RigidBody>? DeactivatedBody;
            public event Action<RigidBody>? ActivatedBody;

            internal WorldEvents() { }

            internal void RaiseWorldPreStep(float timestep)
            {
                PreStep?.Invoke(timestep);
            }

            internal void RaiseWorldPostStep(float timestep)
            {
                PostStep?.Invoke(timestep);
            }

            internal void RaiseAddedRigidBody(RigidBody body)
            {
                AddedRigidBody?.Invoke(body);
            }

            internal void RaiseRemovedRigidBody(RigidBody body)
            {
                RemovedRigidBody?.Invoke(body);
            }

            internal void RaiseAddedConstraint(BaseConstraint constraint)
            {
                AddedConstraint?.Invoke(constraint);
            }

            internal void RaiseRemovedConstraint(BaseConstraint constraint)
            {
                RemovedConstraint?.Invoke(constraint);
            }

            internal void RaiseBodiesBeginCollide(RigidBody body1,RigidBody body2)
            {
                BodiesBeginCollide?.Invoke(body1,body2);
            }

            internal void RaiseBodiesEndCollide(RigidBody body1, RigidBody body2)
            {
                BodiesEndCollide?.Invoke(body1,body2);
            }

            internal void RaiseActivatedBody(RigidBody body)
            {
                ActivatedBody?.Invoke(body);
            }

            internal void RaiseDeactivatedBody(RigidBody body)
            {
                DeactivatedBody?.Invoke(body);
            }

            internal void RaiseContactCreated(Contact contact)
            {
                ContactCreated?.Invoke(contact);
            }
        }

        private readonly IslandManager _islands = new();

        private readonly HashSet<RigidBody> _bodies = new();
        public IReadOnlyCollection<RigidBody> RigidBodies => _bodies;

        private readonly HashSet<BaseConstraint> _constraints = new();
        public IReadOnlyCollection<BaseConstraint> Constraints => _constraints;

        public WorldEvents Events { get; } = new();

        /// <summary>
        /// Holds a list of <see cref="Arbiter"/>. All currently
        /// active arbiter in the <see cref="World"/> are stored in this map.
        /// </summary>
        public ArbiterMap ArbiterMap { get; } = new();

        private readonly Queue<Arbiter> removedArbiterQueue = new();
        private readonly Queue<Arbiter> addedArbiterQueue = new();

        public ContactSettings ContactSettings { get; } = new();

        /// <summary>
        /// Gets a read only collection of the <see cref="Jitter.Collision.CollisionIsland"/> objects managed by
        /// this class.
        /// </summary>
        public ReadOnlyCollection<CollisionIsland> Islands => _islands.Readonly;

        /// <summary>
        /// Gets the <see cref="CollisionSystem"/> used
        /// to detect collisions.
        /// </summary>
        public CollisionSystem CollisionSystem { get; }

        /// <summary>
        /// Gets or sets the gravity in this <see cref="World"/>. The default gravity is (0,0,0)
        /// </summary>
        public Vector3 Gravity { get; set; } = new(0, 0, 0);

        /// <summary>
        /// Global sets or gets if a body is able to be temporarily deactivated by the engine to
        /// save computation time. Use <see cref="DeactivationAngularVelocityThreshold"/>,
        /// <see cref="DeactivationLinearVelocityThreshold"/> and <see cref="DeactivationTime"/> to
        /// set parameters of the deactivation process.
        /// </summary>
        public bool AllowDeactivation { get; set; } = true;

        /// <summary>
        /// Create a new instance of the <see cref="World"/> class.
        /// </summary>
        /// <param name="collision">The collisionSystem which is used to detect
        /// collisions. See for example: <see cref="CollisionSystemSAP"/>
        /// or <see cref="CollisionSystemBrute"/>.
        /// </param>
        public World(CollisionSystem collision)
        {
            CollisionSystem = collision;
            CollisionSystem.CollisionDetected += CollisionDetected;
        }

        /// <summary>
        /// In Jitter many objects get added to stacks after they were used.
        /// If a new object is needed the old object gets removed from the stack
        /// and is reused. This saves some time and also garbage collections.
        /// Calling this method removes all cached objects from all
        /// stacks.
        /// </summary>
        public static void ClearResourcePools()
        {
            IslandManager.Pool.Clear();
            Arbiter.ResetResourcePool();
            Contact.Pool.Clear();
        }

        #region damping
        private float _angularDamping = 0.85f;
        private float _linearDamping = 0.85f;

        /// <summary>
        /// Factor to reduce linear velocity by every second
        /// </summary>
        public float LinearDampingFactor
        {
            get => _linearDamping;
            set
            {
                if (value is < 0.0f or > 1.0f)
                    throw new ArgumentException("Linear damping factor must be between 0.0 and 1.0", nameof(value));
                _linearDamping = value;
            }
        }

        /// <summary>
        /// Factor to reduce angular velocity by every second
        /// </summary>
        public float AngularDampingFactor
        {
            get => _angularDamping;
            set
            {
                if (value is < 0.0f or > 1.0f)
                    throw new ArgumentException("Angular damping factor must be between 0.0 and 1.0", nameof(value));
                _angularDamping = value;
            }
        }
        #endregion

        #region deactivation
        private float _deactivationTime = 2f;
        private float _inactiveLinearThreshold = 0.3f;
        private float _inactiveAngularThreshold = 0.3f;

        /// <summary>
        /// A body will be deactivated if it's angular velocity and linear velocity are below the thresholds for this amount of time.
        /// </summary>
        public float DeactivationTime
        {
            get => _deactivationTime;
            set
            {
                if (value < 0.0f)
                    throw new ArgumentException("Deactivation time threshold must be greater than or equal to zero", nameof(value));
                _deactivationTime = value;
            }
        }
        
        /// <summary>
        /// A body will be deactivated if it's linear velocity is below this threshold for the <see cref="DeactivationTime"/>
        /// </summary>
        public float DeactivationLinearVelocityThreshold
        {
            get => _inactiveLinearThreshold;
            set
            {
                if (value < 0.0f)
                    throw new ArgumentException("Deactivation linear velocity threshold must be greater than or equal to zero", nameof(value));
                _inactiveLinearThreshold = value;
            }
        }

        /// <summary>
        /// A body will be deactivated if it's angular velocity is below this threshold for the <see cref="DeactivationTime"/>
        /// </summary>
        public float DeactivationAngularVelocityThreshold
        {
            get => _inactiveAngularThreshold;
            set
            {
                if (value < 0.0f)
                    throw new ArgumentException("Deactivation angular velocity threshold must be greater than or equal to zero", nameof(value));
                _inactiveAngularThreshold = value;
            }
        }
        #endregion

        #region iterations
        private int _contactIterations = 10;
        private int _smallIterations = 4;

        /// <summary>
        /// Number of contacts iterations. More iterations leads to a more stable simulation but is computationally more expensive.
        /// As a guide, the value should be around 3 to 30.
        /// </summary>
        public int ContactIterations
        {
            get => _contactIterations;
            set
            {
                if (value <= 0)
                    throw new ArgumentException("The number of contact iterations must be larger than zero", nameof(value));
                _contactIterations = value;
            }
        }

        /// <summary>
        /// Number of contacts iterations to use for smaller (two and three constraint) systems.
        /// More iterations leads to a more stable simulation but is computationally more expensive.
        /// As a guide, the value should be around 3 to 30.
        /// </summary>
        public int SmallIterations
        {
            get => _smallIterations;
            set
            {
                if (value <= 0)
                    throw new ArgumentException("The number of small contact iterations must be larger than zero", nameof(value));
                _smallIterations = value;
            }
        }
        #endregion

        #region Add/Remove
        /// <summary>
        /// Adds a <see cref="RigidBody"/> to the world.
        /// </summary>
        /// <param name="body">The body which should be added.</param>
        public void Add(RigidBody body)
        {
            if (body == null)
                throw new ArgumentNullException(nameof(body), "body can't be null.");
            if (_bodies.Contains(body))
                throw new ArgumentException("The body was already added to the world.", nameof(body));

            Events.RaiseAddedRigidBody(body);

            CollisionSystem.AddEntity(body);

            _bodies.Add(body);
        }

        public bool Remove(RigidBody body)
        {
            // remove the body from the world list
            if (!_bodies.Remove(body))
                return false;

            // Remove all connected constraints and arbiters
            foreach (var arbiter in body.arbiters)
            {
                ArbiterMap.Remove(arbiter);
                Events.RaiseBodiesEndCollide(arbiter.Body1, arbiter.Body2);
            }

            foreach (var constraint in body.constraints)
            {
                _constraints.Remove(constraint);
                Events.RaiseRemovedConstraint(constraint);
            }

            // remove the body from the collision system
            CollisionSystem.RemoveEntity(body);

            // remove the body from the island manager
            _islands.RemoveBody(body);

            Events.RaiseRemovedRigidBody(body);

            return true;
        }

        /// <summary>
        /// Add a <see cref="BaseConstraint"/> to the world.
        /// </summary>
        /// <param name="constraint">The constraint which should be removed.</param>
        public void Add(BaseConstraint constraint)
        {
            if (!_constraints.Add(constraint))
                throw new ArgumentException("The constraint was already added to the world.", nameof(constraint));

            _islands.ConstraintCreated(constraint);
            Events.RaiseAddedConstraint(constraint);
        }

        /// <summary>
        /// Add a <see cref="BaseConstraint"/> to the world. Fast, O(1).
        /// </summary>
        /// <param name="constraint">The constraint which should be added.</param>
        /// <returns>True if the constraint was successfully removed.</returns>
        public bool Remove(BaseConstraint constraint)
        {
            if (!_constraints.Remove(constraint))
                return false;

            Events.RaiseRemovedConstraint(constraint);
            _islands.ConstraintRemoved(constraint);

            return true;
        }

        /// <summary>
        /// Removes all objects from the world and removes all memory cached objects.
        /// </summary>
        public void Clear()
        {
            // remove bodies from collision system
            foreach (var body in _bodies)
            {
                CollisionSystem.RemoveEntity(body);

                if (body.island != null)
                {
                    body.island.ClearLists();
                    body.island = null;
                }

                body.connections.Clear();
                body.arbiters.Clear();
                body.constraints.Clear();

                Events.RaiseRemovedRigidBody(body);
            }

            // remove bodies from the world
            _bodies.Clear();

            // remove constraints
            foreach (var constraint in _constraints)
                Events.RaiseRemovedConstraint(constraint);
            _constraints.Clear();

            // remove all islands
            _islands.RemoveAll();

            // delete the arbiters
            ArbiterMap.Clear();
        }
        #endregion

        #region Debug Timing
        private readonly Stopwatch sw = new();

        public enum DebugType
        {
            CollisionDetect, BuildIslands, HandleArbiter, UpdateContacts,
            PreStep, DeactivateBodies, IntegrateForces, Integrate, PostStep, Num
        }

        private readonly double[] _stepTimesMs = new double[(int)DebugType.Num];

        /// <summary>
        /// Time in ms for every part of the <see cref="Step(float)"/> method.
        /// </summary>
        /// <example>int time = DebugTimes[(int)DebugType.CollisionDetect] gives
        /// the amount of time spent on collision detection during the last <see cref="Step(float)"/>.
        /// </example>
        public IReadOnlyList<double> DebugTimes => _stepTimesMs;
        #endregion

        private float currentLinearDampFactor = 1.0f;
        private float currentAngularDampFactor = 1.0f;
        private float _timestep;
        private float _accumulatedTime;

        /// <summary>
        /// Integrates the whole world a timestep further in time.
        /// </summary>
        /// <param name="timestep">The timestep in seconds. 
        /// It should be small as possible to keep the simulation stable.
        /// The physics simulation shouldn't run slower than 60fps.
        /// (timestep=1/60).</param>
        public void Step(float timestep)
        {
            _timestep = timestep;

            if (timestep == 0.0f)
                return;
            if (timestep < 0.0f)
                throw new ArgumentException("The timestep can't be negative.", nameof(timestep));

            currentAngularDampFactor = (float)Math.Pow(_angularDamping, timestep);
            currentLinearDampFactor = (float)Math.Pow(_linearDamping, timestep);

            sw.Restart();
            Events.RaiseWorldPreStep(timestep);
            _stepTimesMs[(int)DebugType.PreStep] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            UpdateContacts();
            _stepTimesMs[(int)DebugType.UpdateContacts] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            while (removedArbiterQueue.Count > 0)
            {
                var arbiter = removedArbiterQueue.Dequeue();
                _islands.ArbiterRemoved(arbiter);
                Arbiter.Destroy(arbiter);
            }
            var ms = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            CollisionSystem.Detect();
            _stepTimesMs[(int)DebugType.CollisionDetect] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            while (addedArbiterQueue.Count > 0)
                _islands.ArbiterCreated(addedArbiterQueue.Dequeue());
            _stepTimesMs[(int)DebugType.BuildIslands] = sw.Elapsed.TotalMilliseconds + ms;

            sw.Restart();
            CheckDeactivation();
            _stepTimesMs[(int)DebugType.DeactivateBodies] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            IntegrateForces();
            _stepTimesMs[(int)DebugType.IntegrateForces] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            HandleArbiter();
            _stepTimesMs[(int)DebugType.HandleArbiter] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            Integrate();
            _stepTimesMs[(int)DebugType.Integrate] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            Events.RaiseWorldPostStep(timestep);
            _stepTimesMs[(int)DebugType.PostStep] = sw.Elapsed.TotalMilliseconds;
        }

        /// <summary>
        /// Integrates the whole world several fixed timestep further in time.
        /// </summary>
        /// <param name="totalTime">The time to integrate.</param>
        /// <param name="timestep">The timestep in seconds. 
        /// It should be small as possible to keep the simulation stable.
        /// The physics simulation shouldn't run slower than 60fps.
        /// (timestep=1/60).</param>
        /// <param name="maxSteps">The maximum number of substeps. After that Jitter gives up
        /// to keep up with the given totalTime.</param>
        public void Step(float totalTime, float timestep, int maxSteps)
        {
            var counter = 0;
            _accumulatedTime += totalTime;

            while (_accumulatedTime > timestep)
            {
                Step(timestep);

                _accumulatedTime -= timestep;
                counter++;

                if (counter > maxSteps)
                {
                    // okay, okay... we can't keep up
                    _accumulatedTime = 0.0f;
                    break;
                }
            }

        }

        private void UpdateArbiterContacts(Arbiter arbiter)
        {
            if (arbiter.ContactList.Count == 0)
            {
                _removedArbiterStack.Push(arbiter);
                return;
            }

            for (var i = arbiter.ContactList.Count - 1; i >= 0; i--)
            {
                var c = arbiter.ContactList[i];
                c.UpdatePosition();

                if (c.penetration < -ContactSettings.BreakThreshold)
                {
                    Contact.Pool.Return(c);
                    arbiter.ContactList.RemoveAt(i);
                }
                else
                {
                    var diff = c.p1 - c.p2;
                    var distance = Vector3.Dot(diff, c.normal);

                    diff -= distance * c.normal;
                    distance = diff.LengthSquared();

                    // hack (multiplication by factor 100) in the
                    // following line.
                    if (distance > ContactSettings.BreakThreshold * ContactSettings.BreakThreshold * 100)
                    {
                        Contact.Pool.Return(c);
                        arbiter.ContactList.RemoveAt(i);
                    }
                }

            }
        }

        private readonly Stack<Arbiter> _removedArbiterStack = new();

        private void UpdateContacts()
        {
            foreach (var arbiter in ArbiterMap.Arbiters)
                UpdateArbiterContacts(arbiter);

            while (_removedArbiterStack.Count > 0)
            {
                var arbiter = _removedArbiterStack.Pop();
                ArbiterMap.Remove(arbiter);

                removedArbiterQueue.Enqueue(arbiter);
                Events.RaiseBodiesEndCollide(arbiter.Body1, arbiter.Body2);
            }
        }

        private void ArbiterCallback(CollisionIsland island)
        {
            var thisIterations = island.Bodies.Count + island.Constraints.Count > 3
                               ? _contactIterations
                               : _smallIterations;

            for (var i = -1; i < thisIterations; i++)
            {
                // Contact and Collision
                foreach (var arbiter in island.arbiter)
                {
                    var contactCount = arbiter.ContactList.Count;
                    for (var e = 0; e < contactCount; e++)
                    {
                        if (i == -1)
                            arbiter.ContactList[e].PrepareForIteration(_timestep);
                        else
                            arbiter.ContactList[e].Iterate();
                    }
                }

                //  Constraints
                foreach (var c in island.constraints)
                {
                    if (c.Body1 is { IsActive: false } && c.Body2 is { IsActive: false })
                        continue;

                    if (i == -1)
                        c.PrepareForIteration(_timestep);
                    else
                        c.Iterate();
                }

            }
        }

        private void HandleArbiter()
        {
            for (var i = 0; i < _islands.Count; i++)
                if (_islands[i].IsActive())
                    ArbiterCallback(_islands[i]);
        }

        private void IntegrateForces()
        {
            foreach (var body in _bodies)
            {
                if (!body.IsStatic && body.IsActive)
                {
                    var scaleFactor = body.inverseMass * _timestep;
                    body.linearVelocity = body.AccumulatedForce * scaleFactor + body.LinearVelocity;

                    {
                        var temp = body.AccumulatedTorque * _timestep;
                        temp = temp.Transform(body.invInertiaWorld);
                        body.AngularVelocity = temp + body.AngularVelocity;
                    }

                    if (body.AffectedByGravity)
                    {
                        var temp = Gravity * _timestep;
                        body.LinearVelocity += temp;
                    }
                }

                body.AccumulatedForce = default;
                body.AccumulatedTorque = default;

            }
        }

        private void IntegrateCallback(RigidBody body)
        {
            body.Position = body.LinearVelocity * _timestep + body.Position;

            //exponential map
            Vector3 axis;
            var angle = body.AngularVelocity.Length();

            if (angle < 0.001f)
            {
                // use Taylor's expansions of sync function
                // axis = body.angularVelocity * (0.5f * timestep - (timestep * timestep * timestep) * (0.020833333333f) * angle * angle);
                var scaleFactor = 0.5f * _timestep - MathF.Pow(_timestep, 3) * 0.020833333333f * MathF.Pow(angle, 2);
                axis = body.AngularVelocity * scaleFactor;
            }
            else
            {
                // sync(fAngle) = sin(c*fAngle)/t
                var scaleFactor = MathF.Sin(0.5f * angle * _timestep) / angle;
                axis = body.AngularVelocity * scaleFactor;
            }

            var dorn = new Quaternion(axis.X, axis.Y, axis.Z, (float)Math.Cos(angle * _timestep * 0.5f));

            var ornA = body.orientation.ToQuaternion();
            dorn = Quaternion.Multiply(dorn, ornA);

            dorn = Quaternion.Normalize(dorn);
            body.orientation = JMatrix.CreateFromQuaternion(dorn);

            if ((body.Damping & RigidBody.DampingType.Linear) != 0)
                body.LinearVelocity *= currentLinearDampFactor;

            if ((body.Damping & RigidBody.DampingType.Angular) != 0)
                body.AngularVelocity *= currentAngularDampFactor;

            body.Update();

            
            if (CollisionSystem.EnableSpeculativeContacts || body.EnableSpeculativeContacts)
                body.SweptExpandBoundingBox(_timestep);
        }


        private void Integrate()
        {
            {
                foreach (var body in _bodies)
                {
                    if (body.IsStatic || !body.IsActive)
                        continue;
                    IntegrateCallback(body);
                }
            }
        }

        private void CollisionDetected(RigidBody body1, RigidBody body2, Vector3 point1, Vector3 point2, Vector3 normal, float penetration)
        {
            ArbiterMap.LookUpArbiter(body1, body2, out var arbiter);
            if (arbiter == null)
            {
                arbiter = ArbiterMap.Add(body1, body2);

                addedArbiterQueue.Enqueue(arbiter);

                Events.RaiseBodiesBeginCollide(body1, body2);
            }

            var contact = arbiter.Body1 == body1
                        ? arbiter.AddContact(point1, point2, -normal, penetration, ContactSettings)
                        : arbiter.AddContact(point2, point1, normal, penetration, ContactSettings);

            if (contact != null)
                Events.RaiseContactCreated(contact);

        }

        private void CheckDeactivation()
        {
            // A body deactivation DOESN'T kill the contacts - they are stored in
            // the arbitermap within the arbiters. So, waking up is STABLE - old
            // contacts are reused. Also the collisionislands build every frame (based 
            // on the contacts) keep the same.

            var inactiveLinearThresholdSq = _inactiveLinearThreshold * _inactiveLinearThreshold;
            var inactiveAngularThresholdSq = _inactiveAngularThreshold * _inactiveAngularThreshold;

            foreach (var island in _islands)
            {
                var deactivateIsland = true;

                if (!AllowDeactivation)
                {
                    deactivateIsland = false;
                }
                else
                {
                    foreach (var body in island.bodies)
                    {
                        if (body.AllowDeactivation && body.AngularVelocity.LengthSquared() < inactiveAngularThresholdSq &&
                            body.linearVelocity.LengthSquared() < inactiveLinearThresholdSq)
                        {
                            body.inactiveTime += _timestep;
                            if (body.inactiveTime < DeactivationTime)
                                deactivateIsland = false;
                        }
                        else
                        {
                            body.inactiveTime = 0.0f;
                            deactivateIsland = false;
                        }
                    }
                }

                foreach (var body in island.bodies)
                {
                    if (body.IsActive == deactivateIsland)
                    {
                        if (body.IsActive)
                        {
                            body.IsActive = false;
                            Events.RaiseDeactivatedBody(body);
                        }
                        else
                        {
                            body.IsActive = true;
                            Events.RaiseActivatedBody(body);
                        }
                    }
                    
                }
            }
        }

    }
}
