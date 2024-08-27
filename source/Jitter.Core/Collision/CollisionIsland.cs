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

using System.Collections.Generic;
using Jitter.Dynamics;
using Jitter.Dynamics.Constraints;

namespace Jitter.Collision
{
    /// <summary>
    /// Holds a list of bodies which are in contact with each other.
    /// </summary>
    public class CollisionIsland
    {

        internal IslandManager islandManager;

        internal readonly HashSet<RigidBody> bodies = new();
        internal readonly HashSet<Arbiter> arbiter = new();
        internal readonly HashSet<Constraint> constraints = new();

        /// <summary>
        /// Gets a read only list of <see cref="RigidBody"/> which are in contact with each other.
        /// </summary>
        public IReadOnlyCollection<RigidBody> Bodies => bodies;

        /// <summary>
        /// Gets a read only list of <see cref="Arbiter"/> which are involved in this island.
        /// </summary>
        public IReadOnlyCollection<Arbiter> Arbiter => arbiter;

        /// <summary>
        /// Gets a read only list of <see cref="Constraint"/> which are involved in this island.
        /// </summary>
        public IReadOnlyCollection<Constraint> Constraints => constraints;

        /// <summary>
        /// Whether the island is active or not.
        /// </summary>
        /// <returns>Returns true if the island is active, otherwise false.</returns>
        /// <seealso cref="RigidBody.IsActive"/>
        public bool IsActive()
        {
            var enumerator = bodies.GetEnumerator();
            enumerator.MoveNext();

            if (enumerator.Current == null) return false;
            else return enumerator.Current.isActive;
        }

        /// <summary>
        /// Sets the status of every body in this island to active or inactive.
        /// </summary>
        /// <param name="active">If true the island gets activated, if false it
        /// gets deactivated. </param>
        /// <seealso cref="RigidBody.IsActive"/>
        public void SetStatus(bool active)
        {
            foreach (var body in bodies)
            {
                body.IsActive = active;
                if (active && !body.IsActive) body.inactiveTime = 0.0f;
            }

        }

        internal void ClearLists()
        {
            arbiter.Clear(); bodies.Clear(); constraints.Clear();
        }

    }
}
