﻿/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
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
using System.Threading;

namespace Jitter.Dynamics.Constraints
{
    /// <summary>
    /// A constraints forces a body to behave in a specific way.
    /// </summary>
    public abstract class BaseConstraint
        : IConstraint, IDebugDrawable, IComparable<BaseConstraint>
    {
        /// <summary>
        /// Gets the first body. Can be null.
        /// </summary>
        public RigidBody Body1 { get; }

        /// <summary>
        /// Gets the second body. Can be null.
        /// </summary>
        public RigidBody Body2 { get; }

        private static int instanceCount;
        private readonly int instance;

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="body1">The first body which should get constrained. Can be null.</param>
        /// <param name="body2">The second body which should get constrained. Can be null.</param>
        public BaseConstraint(RigidBody body1, RigidBody body2)
        {
            Body1 = body1;
            Body2 = body2;

            instance = Interlocked.Increment(ref instanceCount);

            // calling body.update does not hurt
            // if the user set orientations all
            // inverse orientations etc. get also
            // recalculated.
            body1?.Update();
            body2?.Update();
        }

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The simulation timestep</param>
        public abstract void PrepareForIteration(float timestep);

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public abstract void Iterate();


        public int CompareTo(BaseConstraint other)
        {
            return instance.CompareTo(other.instance);
        }

        public virtual void DebugDraw(IDebugDrawer drawer)
        {
        }
    }
}
