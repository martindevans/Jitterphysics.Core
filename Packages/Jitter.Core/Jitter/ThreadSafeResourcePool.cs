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

#nullable enable

using System;
using System.Collections.Generic;

namespace Jitter
{
    /// <summary>
    /// A thread safe resource pool.
    /// </summary>
    /// <typeparam name="T">The type of the object to cache. The type T must
    /// have a parameterless constructor.</typeparam>
    public class ThreadSafeResourcePool<T>
        where T : class
    {
        private readonly Func<T> create;
        private readonly Stack<T> stack = new();

        /// <summary>
        /// Creates a new instance of the ThreadSafeResourcePool class.
        /// </summary>
        public ThreadSafeResourcePool(Func<T>? create = null)
        {
            this.create = create ?? Activator.CreateInstance<T>;
        }

        /// <summary>
        /// Removes all cached resources.
        /// So they can get garbage collected.
        /// </summary>
        public void Clear()
        {
            lock (stack)
            {
                stack.Clear();
            }
        }

        /// <summary>
        /// Gives a resource back to the pool.
        /// </summary>
        /// <param name="obj">The resource to give back</param>
        public void Return(T obj)
        {
            lock (stack)
            {
                stack.Push(obj);
            }
        }

        /// <summary>
        /// Get a free resource.
        /// </summary>
        /// <returns>The free resource.</returns>
        public T Take()
        {
            lock (stack)
            {
                if (stack.Count > 0)
                    return stack.Pop();
            }

            return create.Invoke();
        }
    }
}
