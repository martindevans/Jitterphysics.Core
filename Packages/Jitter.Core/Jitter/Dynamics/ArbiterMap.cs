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
using System.Collections.Generic;

namespace Jitter.Dynamics
{
    /// <summary>
    /// For easy access, Arbiters are stored in a Hashtable(ArbiterMap). 
    /// To find the Arbiter fortwo RigidBodies, build an ArbiterKey for the two bodies
    /// and use it as the lookup key for the ArbiterMap.
    /// </summary>
    public readonly struct ArbiterKey
        : IEquatable<ArbiterKey>
    {
        public readonly RigidBody Body1;
        public readonly RigidBody Body2;

        /// <summary>
        /// Initializes a new instance of the ArbiterKey class.
        /// </summary>
        /// <param name="body1"></param>
        /// <param name="body2"></param>
        public ArbiterKey(RigidBody body1, RigidBody body2)
        {
            Body1 = body1;
            Body2 = body2;
        }

        public bool Equals(ArbiterKey other)
        {
            return other.Body1.Equals(Body1) && other.Body2.Equals(Body2) ||
                   other.Body1.Equals(Body2) && other.Body2.Equals(Body1);
        }

        /// <summary>
        /// Checks if two objects are equal.
        /// </summary>
        /// <param name="obj">The object to check against.</param>
        /// <returns>Returns true if they are equal, otherwise false.</returns>
        public override bool Equals(object obj)
        {
            if (obj is not ArbiterKey key)
                return false;
            return Equals(key);
        }

        /// <summary>
        /// Returns the hashcode of the ArbiterKey.
        /// The hashcode is the same if an ArbiterKey contains the same bodies.
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            // This hash is intentionally order invariant!
            return unchecked(Body1.GetHashCode() + Body2.GetHashCode());
        }
    }

    /// <summary>
    /// The ArbiterMap is a dictionary which stores all arbiters.
    /// </summary>
    public class ArbiterMap
    {
        private readonly Dictionary<ArbiterKey, Arbiter> dictionary = new();

        /// <summary>
        /// Gets an arbiter by it's bodies.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <param name="arbiter">The arbiter which was found.</param>
        /// <returns>Returns true if the arbiter could be found, otherwise false.</returns>
        public bool LookUpArbiter(RigidBody body1, RigidBody body2, out Arbiter arbiter)
        {
            var key = new ArbiterKey(body1, body2);
            return dictionary.TryGetValue(key, out arbiter);
        }

        public Dictionary<ArbiterKey, Arbiter>.ValueCollection Arbiters => dictionary.Values;

        internal void Add(ArbiterKey key, Arbiter arbiter)
        {
            dictionary.Add(key, arbiter);
        }

        internal void Clear()
        {
            dictionary.Clear();
        }

        internal bool Remove(Arbiter arbiter)
        {
            return dictionary.Remove(new(arbiter.Body1, arbiter.Body2));
        }

        /// <summary>
        /// Checks if an arbiter is within the arbiter map.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <returns>Returns true if the arbiter could be found, otherwise false.</returns>
        public bool ContainsArbiter(RigidBody body1, RigidBody body2)
        {
            var key = new ArbiterKey(body1, body2);
            return dictionary.ContainsKey(key);
        }
    }

}
