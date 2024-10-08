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

using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using Jitter.LinearMath;

namespace Jitter.Collision.Shapes
{


    /// <summary>
    /// Represents a variable form of a shape.
    /// </summary>
    public abstract class Multishape : BaseShape
    {

        /// <summary>
        /// Sets the current shape. First <see cref="Prepare"/> has to be called.
        /// After SetCurrentShape the shape immitates another shape.
        /// </summary>
        /// <param name="index"></param>
        public abstract void SetCurrentShape(int index);

        /// <summary>
        /// Passes a axis aligned bounding box to the shape where collision
        /// could occour.
        /// </summary>
        /// <param name="box">The bounding box where collision could occur.</param>
        /// <returns>The upper index with which <see cref="SetCurrentShape"/> can be 
        /// called.</returns>
        public abstract int Prepare(ref JBBox box);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayDelta"></param>
        /// <returns></returns>
        public abstract int Prepare(ref Vector3 rayOrigin, ref Vector3 rayDelta);

        protected abstract Multishape CreateWorkingClone();

        public bool IsClone { get; private set; }

        private Stack<Multishape> workingCloneStack = new();
        public Multishape RequestWorkingClone()
        {
            Debug.Assert(workingCloneStack.Count<10, "Unusual size of the workingCloneStack. Forgot to call ReturnWorkingClone?");
            Debug.Assert(!IsClone, "Can't clone clones! Something wrong here!");

            Multishape multiShape;

            if (workingCloneStack.Count == 0)
            {
                multiShape = CreateWorkingClone();
                multiShape.workingCloneStack = workingCloneStack;
                workingCloneStack.Push(multiShape);
            }
            multiShape = workingCloneStack.Pop();
            multiShape.IsClone = true;

            return multiShape;
        }

        public override void UpdateShape()
        {
            workingCloneStack.Clear();
            base.UpdateShape();
        }

        public void ReturnWorkingClone()
        {
            Debug.Assert(IsClone, "Only clones can be returned!");
            workingCloneStack.Push(this);
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape. This includes
        /// the whole shape.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <returns>The axis aligned bounding box of the shape.</returns>
        public override JBBox GetBoundingBox(JMatrix orientation)
        {
            var helpBox = JBBox.LargeBox;
            var length = Prepare(ref helpBox);

            var box = JBBox.SmallBox;

            for (var i = 0; i < length; i++)
            {
                SetCurrentShape(i);
                helpBox = base.GetBoundingBox(orientation);
                box = JBBox.CreateMerged(in box, in helpBox);
            }

            return box;
        }

        public override void MakeHull(List<Vector3> triangleList, int generationThreshold)
        {
            //throw new NotImplementedException();
        }


        /// <summary>
        /// Calculates the inertia of a box with the sides of the multishape.
        /// </summary>
        public override void CalculateMassInertia()
        {
            geomCen = default;

            // TODO: calc this right
            var i = JMatrix.Identity;

            var size = boundingBox.Max - boundingBox.Min;

            Mass = size.X * size.Y * size.Z;

            i.M11 = 1.0f / 12.0f * Mass * (size.Y * size.Y + size.Z * size.Z);
            i.M22 = 1.0f / 12.0f * Mass * (size.X * size.X + size.Z * size.Z);
            i.M33 = 1.0f / 12.0f * Mass * (size.X * size.X + size.Y * size.Y);

            Inertia = i;
        }

    }
}
