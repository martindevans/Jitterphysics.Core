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
using System.Diagnostics;
using System.Numerics;

namespace Jitter.Dynamics
{
    /// <summary>
    /// A list of contacts. Maximum of 4 contacts.
    /// </summary>
    public class ContactList
    {
        private readonly List<Contact> _contacts = new List<Contact>(4);

        public int Count => _contacts.Count;
        public Contact this[int index] => _contacts[index];

        internal void Add(Contact contact)
        {
            if (_contacts.Count >= 4)
                throw new InvalidOperationException("Cannot store more than 4 contacts");

            _contacts.Add(contact);
        }

        internal void Clear()
        {
            _contacts.Clear();
        }

        internal void RemoveAt(int index)
        {
            _contacts.RemoveAt(index);
        }
    }

    /// <summary>
    /// An arbiter holds all contact information of two bodies.
    /// The contacts are stored in the ContactList. There is a maximum
    /// of four contacts which can be added to an arbiter. The arbiter
    /// only keeps the best four contacts based on the area spanned by
    /// the contact points.
    /// </summary>
    public class Arbiter
    {
        /// <summary>
        /// </summary>
        private static readonly ThreadSafeResourcePool<Arbiter> Pool = new(() => new Arbiter());

        /// <summary>
        /// The first body.
        /// </summary>
        public RigidBody Body1 { get; private set; }

        /// <summary>
        /// The second body.
        /// </summary>
        public RigidBody Body2 { get; private set; }

        /// <summary>
        /// The contact list containing all contacts of both bodies.
        /// </summary>
        public ContactList ContactList { get; } = new();

        private Arbiter()
        {
            Body1 = null!;
            Body2 = null!;
        }

        internal static Arbiter Create(RigidBody body1, RigidBody body2)
        {
            var a = Pool.GetNew();
            a.Body1 = body1;
            a.Body2 = body2;
            return a;
        }

        internal static void Destroy(Arbiter arbiter)
        {
            arbiter.Body1 = null!;
            arbiter.Body2 = null!;
            Pool.GiveBack(arbiter);
        }

        internal static void ResetResourcePool()
        {
            Pool.ResetResourcePool();
        }

        /// <summary>
        /// Removes all contacts from this arbiter.
        /// The world will remove the arbiter automatically next frame
        /// or add new contacts.
        /// </summary>
        public void Invalidate()
        {
            ContactList.Clear();
        }

        /// <summary>
        /// Adds a contact to the arbiter. No more than four contacts 
        /// are stored in the contactList. When adding a new contact
        /// to the arbiter the existing are checked and the best 4 are kept.
        /// </summary>
        /// <param name="point1">Point on body1. In world space.</param>
        /// <param name="point2">Point on body2. In world space.</param>
        /// <param name="normal">The normal pointing to body2.</param>
        /// <param name="penetration">The estimated penetration depth.</param>
        /// <param name="contactSettings"></param>
        public Contact? AddContact(Vector3 point1, Vector3 point2, Vector3 normal, float penetration, ContactSettings contactSettings)
        {
            var relPos1 = point1 - Body1.position;

            int index;
            if (ContactList.Count == 4)
            {
                index = SortCachedPoints(ref relPos1, penetration);
                ReplaceContact(ref point1, ref point2, ref normal, penetration, index, contactSettings);
                return null;
            }

            index = GetCacheEntry(ref relPos1, contactSettings.BreakThreshold);

            if (index >= 0)
            {
                ReplaceContact(ref point1, ref point2, ref normal, penetration, index, contactSettings);
                return null;
            }
            else
            {
                var contact = Contact.Pool.GetNew();
                contact.Initialize(Body1, Body2, ref point1, ref point2, ref normal, penetration, true, contactSettings);
                ContactList.Add(contact);
                return contact;
            }
        }

        private void ReplaceContact(ref Vector3 point1, ref Vector3 point2, ref Vector3 n, float p, int index, ContactSettings contactSettings)
        {
            var contact = ContactList[index];

            Debug.Assert(Body1 == contact.body1, "Body1 and Body2 not consistent.");

            contact.Initialize(Body1, Body2, ref point1, ref point2, ref n, p, false, contactSettings);

        }

        private int GetCacheEntry(ref Vector3 realRelPos1, float contactBreakThreshold)
        {
            var shortestDist = contactBreakThreshold * contactBreakThreshold;
            var size = ContactList.Count;
            var nearestPoint = -1;
            for (var i = 0; i < size; i++)
            {
                var diffA = ContactList[i].relativePos1 - realRelPos1;
                var distToManiPoint = diffA.LengthSquared();
                if (distToManiPoint < shortestDist)
                {
                    shortestDist = distToManiPoint;
                    nearestPoint = i;
                }
            }

            return nearestPoint;
        }

        // sort cached points so most isolated points come first
        private int SortCachedPoints(ref Vector3 realRelPos1, float pen)
        {
            //calculate 4 possible cases areas, and take biggest area
            //also need to keep 'deepest'

            var maxPenetrationIndex = -1;
            var maxPenetration = pen;
            for (var i = 0; i < 4; i++)
            {
                if (ContactList[i].penetration > maxPenetration)
                {
                    maxPenetrationIndex = i;
                    maxPenetration = ContactList[i].penetration;
                }
            }

            float res0 = 0, res1 = 0, res2 = 0, res3 = 0;
            if (maxPenetrationIndex != 0)
            {
                var value2 = ContactList[1].relativePos1;
                var a0 = realRelPos1 - value2;
                var value3 = ContactList[2].relativePos1;
                var b0 = ContactList[3].relativePos1 - value3;
                var cross = Vector3.Cross(a0, b0);
                res0 = cross.LengthSquared();
            }

            if (maxPenetrationIndex != 1)
            {
                var value2 = ContactList[0].relativePos1;
                var a0 = realRelPos1 - value2;
                var value3 = ContactList[2].relativePos1;
                var b0 = ContactList[3].relativePos1 - value3;
                var cross = Vector3.Cross(a0, b0);
                res1 = cross.LengthSquared();
            }

            if (maxPenetrationIndex != 2)
            {
                var value2 = ContactList[0].relativePos1;
                var a0 = realRelPos1 - value2;
                var value3 = ContactList[1].relativePos1;
                var b0 = ContactList[3].relativePos1 - value3;
                var cross = Vector3.Cross(a0, b0);
                res2 = cross.LengthSquared();
            }

            if (maxPenetrationIndex != 3)
            {
                var value2 = ContactList[0].relativePos1;
                var a0 = realRelPos1 - value2;
                var value3 = ContactList[1].relativePos1;
                var b0 = ContactList[2].relativePos1 - value3;
                var cross = Vector3.Cross(a0, b0);
                res3 = cross.LengthSquared();
            }

            var biggestarea = MaxAxis(res0, res1, res2, res3);
            return biggestarea;
        }

        private static int MaxAxis(float x, float y, float z, float w)
        {
            // Assume x is max
            var maxIndex = 0;
            var maxVal = x;

            // Check the others
            if (y > maxVal)
            {
                maxIndex = 1;
                maxVal = y;
            }

            if (z > maxVal)
            {
                maxIndex = 2;
                maxVal = z;
            }

            if (w > maxVal)
            {
                maxIndex = 3;
                maxVal = w;
            }

            return maxIndex;
        }
    }
}
