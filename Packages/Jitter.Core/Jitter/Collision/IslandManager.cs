using System.Collections.Generic;
using Jitter.Dynamics;
using Jitter.Dynamics.Constraints;
using System.Collections.ObjectModel;

namespace Jitter.Collision
{
    /// <summary>
    /// bodies have: connections - bodies they are connected with (via constraints or arbiters)
    ///              arbiters    - all arbiters they are involved
    ///              constraints - all constraints they are involved
    ///              
    /// static bodies dont have any connections. Think of the islands as a graph:
    /// nodes are the bodies, and edges are the connections
    /// </summary>
    internal sealed class IslandManager
    {
        public static readonly ThreadSafeResourcePool<CollisionIsland> Pool = new();

        private readonly List<CollisionIsland> _islands = new();

        public CollisionIsland this[int index] => _islands[index];
        public int Count => _islands.Count;
        public ReadOnlyCollection<CollisionIsland> Readonly { get; }

        private readonly Stack<RigidBody> _rmStackRb = new();
        private readonly Stack<Arbiter> _rmStackArb = new();
        private readonly Stack<BaseConstraint> _rmStackCstr = new();

        private readonly Queue<RigidBody> _leftSearchQueue = new();
        private readonly Queue<RigidBody> _rightSearchQueue = new();
        private readonly List<RigidBody> _visitedBodiesLeft = new();
        private readonly List<RigidBody> _visitedBodiesRight = new();

        public IslandManager()
        {
            Readonly = new ReadOnlyCollection<CollisionIsland>(_islands);
        }

        public void ArbiterCreated(Arbiter arbiter)
        {
            AddConnection(arbiter.Body1, arbiter.Body2);

            arbiter.Body1.arbiters.Add(arbiter);
            arbiter.Body2.arbiters.Add(arbiter);

            if (arbiter.Body1.island != null)
                arbiter.Body1.island.arbiter.Add(arbiter);
            else
                arbiter.Body2.island?.arbiter.Add(arbiter);
        }

        public void ArbiterRemoved(Arbiter arbiter)
        {
            arbiter.Body1.arbiters.Remove(arbiter);
            arbiter.Body2.arbiters.Remove(arbiter);

            if (arbiter.Body1.island != null)
                arbiter.Body1.island.arbiter.Remove(arbiter);
            else
                arbiter.Body2.island?.arbiter.Remove(arbiter);

            RemoveConnection(arbiter.Body1, arbiter.Body2);
        }

        public void ConstraintCreated(BaseConstraint constraint)
        {
            AddConnection(constraint.Body1, constraint.Body2);

            constraint.Body1.constraints.Add(constraint);
            constraint.Body2?.constraints.Add(constraint);

            if (constraint.Body1.island != null)
                constraint.Body1.island.constraints.Add(constraint);
            else if (constraint.Body2 is { island: not null })
                constraint.Body2.island.constraints.Add(constraint);
        }

        public void ConstraintRemoved(BaseConstraint constraint)
        {
            constraint.Body1.constraints.Remove(constraint);

            constraint.Body2?.constraints.Remove(constraint);

            if (constraint.Body1.island != null)
                constraint.Body1.island.constraints.Remove(constraint);
            else if (constraint.Body2 is { island: not null })
                constraint.Body2.island.constraints.Remove(constraint);

            RemoveConnection(constraint.Body1, constraint.Body2);
        }

        public void MakeBodyStatic(RigidBody body)
        {

            foreach (var b in body.connections) _rmStackRb.Push(b);
            while (_rmStackRb.Count > 0) RemoveConnection(body,_rmStackRb.Pop());

            // A static body doesn't have any connections.
            body.connections.Clear();

            if (body.island != null)
            {
                body.island.bodies.Remove(body);

                if (body.island.bodies.Count == 0)
                {
                    body.island.ClearLists();
                    Pool.Return(body.island);
                }
            }

            body.island = null;
        }

        public void RemoveBody(RigidBody body)
        {
            // Remove everything.
            foreach (var arbiter in body.arbiters) _rmStackArb.Push(arbiter);
            while (_rmStackArb.Count > 0) ArbiterRemoved(_rmStackArb.Pop());

            foreach (var constraint in body.constraints) _rmStackCstr.Push(constraint);
            while (_rmStackCstr.Count > 0) ConstraintRemoved(_rmStackCstr.Pop());

            body.arbiters.Clear();
            body.constraints.Clear();

            if (body.island != null)
            {
                System.Diagnostics.Debug.Assert(body.island.islandManager == this,
                    "IslandManager Inconsistency: IslandManager doesn't own the Island.");


                // the body should now form an island on his own.
                // thats okay, but since static bodies dont have islands
                // remove this island.
                System.Diagnostics.Debug.Assert(body.island.bodies.Count == 1,
                "IslandManager Inconsistency: Removed all connections of a body - body is still in a non single Island.");

                body.island.ClearLists();
                Pool.Return(body.island);

                _islands.Remove(body.island);

                body.island = null;
            }

        }


        public void RemoveAll()
        {
            foreach (var island in _islands)
            {
                foreach (var body in island.bodies)
                {
                    body.arbiters.Clear();
                    body.constraints.Clear();
                    body.connections.Clear();
                    body.island = null;
                }
                island.ClearLists();
            }
            _islands.Clear();
        }

        private void AddConnection(RigidBody body1, RigidBody body2)
        {
            System.Diagnostics.Debug.Assert(!(body1.IsStatic && body2.IsStatic),
                "IslandManager Inconsistency: Arbiter detected between two static objects.");

            if (body1.IsStatic) // <- only body1 is static
            {
                if (body2.island == null)
                {
                    var newIsland = Pool.Take();
                    newIsland.islandManager = this;

                    body2.island = newIsland;
                    body2.island.bodies.Add(body2);
                    _islands.Add(newIsland);
                }
            }
            else if (body2 == null || body2.IsStatic) // <- only body2 is static
            {
                if (body1.island == null)
                {
                    var newIsland = Pool.Take();
                    newIsland.islandManager = this;

                    body1.island = newIsland;
                    body1.island.bodies.Add(body1);
                    _islands.Add(newIsland);
                }
            }
            else // both are !static
            {
                MergeIslands(body1, body2);

                body1.connections.Add(body2);
                body2.connections.Add(body1);
            }
        }

        private void RemoveConnection(RigidBody body1, RigidBody body2)
        {
            System.Diagnostics.Debug.Assert(!(body1.IsStatic && body2.IsStatic),
                "IslandManager Inconsistency: Arbiter detected between two static objects.");

            if (body1.IsStatic) // <- only body1 is static
            {
                // if (!body2.connections.Contains(body1)) throw new Exception();
                //System.Diagnostics.Debug.Assert(body2.connections.Contains(body1),
                //    "IslandManager Inconsistency.",
                //    "Missing body in connections.");

                body2.connections.Remove(body1);
            }
            else if (body2 == null || body2.IsStatic) // <- only body2 is static
            {
                //System.Diagnostics.Debug.Assert(body1.connections.Contains(body2),
                //    "IslandManager Inconsistency.",
                //    "Missing body in connections.");

                body1.connections.Remove(body2);
            }
            else // <- both are !static
            {
                System.Diagnostics.Debug.Assert(body1.island == body2.island,
                    "IslandManager Inconsistency: Removing arbiter with different islands.");

                body1.connections.Remove(body2);
                body2.connections.Remove(body1);

                SplitIslands(body1, body2);
            }
        }

        
        private void SplitIslands(RigidBody body0, RigidBody body1)
        {
            System.Diagnostics.Debug.Assert(body0.island != null && body0.island == body1.island,
                "Islands not the same or null.");

            _leftSearchQueue.Enqueue(body0);
            _rightSearchQueue.Enqueue(body1);

            _visitedBodiesLeft.Add(body0);
            _visitedBodiesRight.Add(body1);

            body0.marker = 1;
            body1.marker = 2;

            while (_leftSearchQueue.Count > 0 && _rightSearchQueue.Count > 0)
            {
                var currentNode = _leftSearchQueue.Dequeue();
                if (!currentNode.IsStatic)
                {
                    for (var i = 0; i < currentNode.connections.Count; i++)
                    {
                        var connectedNode = currentNode.connections[i];

                        if (connectedNode.marker == 0)
                        {
                            _leftSearchQueue.Enqueue(connectedNode);
                            _visitedBodiesLeft.Add(connectedNode);
                            connectedNode.marker = 1;
                        }
                        else if (connectedNode.marker == 2)
                        {
                            _leftSearchQueue.Clear();
                            _rightSearchQueue.Clear();
                            goto ResetSearchStates;
                        }
                    }
                }

                currentNode = _rightSearchQueue.Dequeue();
                if (!currentNode.IsStatic)
                {

                    for (var i = 0; i < currentNode.connections.Count; i++)
                    {
                        var connectedNode = currentNode.connections[i];

                        if (connectedNode.marker == 0)
                        {
                            _rightSearchQueue.Enqueue(connectedNode);
                            _visitedBodiesRight.Add(connectedNode);
                            connectedNode.marker = 2;
                        }
                        else if (connectedNode.marker == 1)
                        {
                            _leftSearchQueue.Clear();
                            _rightSearchQueue.Clear();
                            goto ResetSearchStates;
                        }
                    }
                }
            }

            var island = Pool.Take();
            island.islandManager = this;

            _islands.Add(island);

            if (_leftSearchQueue.Count == 0)
            {
                for (var i = 0; i < _visitedBodiesLeft.Count; i++)
                {
                    var body = _visitedBodiesLeft[i];
                    body1.island.bodies.Remove(body);
                    island.bodies.Add(body);
                    body.island = island;

                    foreach (var a in body.arbiters)
                    {
                        body1.island.arbiter.Remove(a);
                        island.arbiter.Add(a);
                    }

                    foreach (var c in body.constraints)
                    {
                        body1.island.constraints.Remove(c);
                        island.constraints.Add(c);
                    }
                }

                _rightSearchQueue.Clear();
            }
            else if (_rightSearchQueue.Count == 0)
            {
                for (var i = 0; i < _visitedBodiesRight.Count; i++)
                {
                    var body = _visitedBodiesRight[i];
                    body0.island.bodies.Remove(body);
                    island.bodies.Add(body);
                    body.island = island;

                    foreach (var a in body.arbiters)
                    {
                        body0.island.arbiter.Remove(a);
                        island.arbiter.Add(a);
                    }

                    foreach (var c in body.constraints)
                    {
                        body0.island.constraints.Remove(c);
                        island.constraints.Add(c);
                    }
                }

                _leftSearchQueue.Clear();
            }

        ResetSearchStates:

            for (var i = 0; i < _visitedBodiesLeft.Count; i++)
            {
                _visitedBodiesLeft[i].marker = 0;
            }

            for (var i = 0; i < _visitedBodiesRight.Count; i++)
            {
                _visitedBodiesRight[i].marker = 0;
            }

            _visitedBodiesLeft.Clear();
            _visitedBodiesRight.Clear();
        }

        // Boths bodies must be !static
        private void MergeIslands(RigidBody body0, RigidBody body1)
        {
            if (body0.island != body1.island) // <- both bodies are in different islands
            {
                if (body0.island == null) // <- one island is null
                {
                    body0.island = body1.island;
                    body0.island.bodies.Add(body0);
                }
                else if (body1.island == null)  // <- one island is null
                {
                    body1.island = body0.island;
                    body1.island.bodies.Add(body1);
                }
                else // <- both islands are different,
                {
                    // merge smaller into larger
                    RigidBody smallIslandOwner, largeIslandOwner;

                    if (body0.island.bodies.Count > body1.island.bodies.Count)
                    {
                        smallIslandOwner = body1;
                        largeIslandOwner = body0;
                    }
                    else
                    {
                        smallIslandOwner = body0;
                        largeIslandOwner = body1;
                    }

                    var giveBackIsland = smallIslandOwner.island;

                    Pool.Return(giveBackIsland);
                    _islands.Remove(giveBackIsland);

                    foreach (var b in giveBackIsland.bodies)
                    {
                        b.island = largeIslandOwner.island;
                        largeIslandOwner.island.bodies.Add(b);
                    }

                    foreach (var a in giveBackIsland.arbiter)
                    {
                        largeIslandOwner.island.arbiter.Add(a);
                    }

                    foreach (var c in giveBackIsland.constraints)
                    {
                        largeIslandOwner.island.constraints.Add(c);
                    }

                    giveBackIsland.ClearLists();
                }

            }
            else if (body0.island == null) // <- both are null
            {
                var island = Pool.Take();
                island.islandManager = this;

                body0.island = body1.island = island;

                body0.island.bodies.Add(body0);
                body0.island.bodies.Add(body1);

                _islands.Add(island);
            }

        }


        public List<CollisionIsland>.Enumerator GetEnumerator()
        {
            return _islands.GetEnumerator();
        }

    }
}
