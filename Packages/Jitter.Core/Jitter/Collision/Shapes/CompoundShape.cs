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
using System.Collections.Generic;
using System.Numerics;
using Jitter.LinearMath;

namespace Jitter.Collision.Shapes
{

    /// <summary>
    /// A <see cref="BaseShape"/> representing a compoundShape consisting
    /// of several 'sub' shapes.
    /// </summary>
    public class CompoundShape
        : Multishape
    {
        /// <summary>
        /// Holds a 'sub' shape and it's transformation. This TransformedShape can
        /// be added to the <see cref="CompoundShape"/>
        /// </summary>
        public struct TransformedShape
        {
            internal Vector3 position;
            internal JMatrix orientation;
            internal JMatrix invOrientation;
            internal JBBox boundingBox;

            /// <summary>
            /// The 'sub' shape.
            /// </summary>
            public BaseShape Shape { get; set; }

            /// <summary>
            /// The position of a 'sub' shape
            /// </summary>
            public Vector3 Position
            {
                readonly get => position;
                set
                {
                    position = value;
                    UpdateBoundingBox();
                }
            }

            public readonly JBBox BoundingBox => boundingBox;

            /// <summary>
            /// The inverse orientation of the 'sub' shape.
            /// </summary>
            public readonly JMatrix InverseOrientation => invOrientation;

            /// <summary>
            /// The orienation of the 'sub' shape.
            /// </summary>
            public JMatrix Orientation
            {
                readonly get => orientation;
                set { orientation = value; JMatrix.Transpose(ref orientation, out invOrientation); UpdateBoundingBox(); }
            }

            public void UpdateBoundingBox()
            {
                boundingBox = Shape.GetBoundingBox(orientation);

                boundingBox.Min += position;
                boundingBox.Max += position;
            }

            /// <summary>
            /// Creates a new instance of the TransformedShape struct.
            /// </summary>
            /// <param name="shape">The shape.</param>
            /// <param name="orientation">The orientation this shape should have.</param>
            /// <param name="position">The position this shape should have.</param>
            public TransformedShape(BaseShape shape, JMatrix orientation, Vector3 position)
            {
                this.position = position;
                this.orientation = orientation;
                JMatrix.Transpose(ref orientation, out invOrientation);
                this.Shape = shape;
                boundingBox = new();
                UpdateBoundingBox();
            }
        }

        private TransformedShape[] shapes;

        /// <summary>
        /// An array conaining all 'sub' shapes and their transforms.
        /// </summary>
        public TransformedShape[] Shapes => shapes;

        private Vector3 shifted;
        public Vector3 Shift => -1.0f * shifted;

        private JBBox mInternalBBox;

        /// <summary>
        /// Created a new instance of the CompountShape class.
        /// </summary>
        /// <param name="shapes">The 'sub' shapes which should be added to this 
        /// class.</param>
        public CompoundShape(List<TransformedShape> shapes)
        {
            this.shapes = new TransformedShape[shapes.Count];
            shapes.CopyTo(this.shapes);

            if (!TestValidity()) 
                throw new ArgumentException("Multishapes are not supported!");

            UpdateShape();
        }

        public CompoundShape(TransformedShape[] shapes)
        {
            this.shapes = new TransformedShape[shapes.Length];
            Array.Copy(shapes, this.shapes, shapes.Length);

            if (!TestValidity())
                throw new ArgumentException("Multishapes are not supported!");

            UpdateShape();
        }

        private bool TestValidity()
        {
            for (var i = 0; i < shapes.Length; i++)
            {
                if (shapes[i].Shape is Multishape) return false;
            }

            return true;
        }

        public override void MakeHull(List<Vector3> triangleList, int generationThreshold)
        {
            var triangles = new List<Vector3>();

            for (var i = 0; i < shapes.Length; i++)
            {
                shapes[i].Shape.MakeHull(triangles, 4);
                for (var e = 0; e < triangles.Count; e++)
                {
                    var pos = triangles[e];
                    var matrix = shapes[i].orientation;
                    pos = pos.Transform(matrix);
                    var value2 = shapes[i].position;
                    pos += value2;
                    triangleList.Add(pos);
                }
                triangles.Clear();
            }
        }

        /// <summary>
        /// Translate all subshapes in the way that the center of mass is
        /// in (0,0,0)
        /// </summary>
        private void DoShifting()
        {
            for (var i = 0; i < Shapes.Length; i++) shifted += Shapes[i].position;
            shifted *= 1.0f / shapes.Length;

            for (var i = 0; i < Shapes.Length; i++) Shapes[i].position -= shifted;
        }

        public override void CalculateMassInertia()
        {
            var inertia = default(JMatrix);
            Mass = 0.0f;

            for (var i = 0; i < Shapes.Length; i++)
            {
                var currentInertia = Shapes[i].InverseOrientation * Shapes[i].Shape.Inertia * Shapes[i].Orientation;
                var p = Shapes[i].Position * -1.0f;
                var m = Shapes[i].Shape.Mass;

                currentInertia.M11 += m * (p.Y * p.Y + p.Z * p.Z);
                currentInertia.M22 += m * (p.X * p.X + p.Z * p.Z);
                currentInertia.M33 += m * (p.X * p.X + p.Y * p.Y);

                currentInertia.M12 += -p.X * p.Y * m;
                currentInertia.M21 += -p.X * p.Y * m;

                currentInertia.M31 += -p.X * p.Z * m;
                currentInertia.M13 += -p.X * p.Z * m;

                currentInertia.M32 += -p.Y * p.Z * m;
                currentInertia.M23 += -p.Y * p.Z * m;

                inertia += currentInertia;
                Mass += m;
            }

            Inertia = inertia;
        }


        internal CompoundShape()
        {
        }

        protected override Multishape CreateWorkingClone()
        {
            var clone = new CompoundShape
            {
                shapes = shapes,
            };
            return clone;
        }


        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public override Vector3 SupportMapping(Vector3 direction)
        {
            var matrix = shapes[currentShape].invOrientation;
            var result = direction.Transform(matrix);
            result = shapes[currentShape].Shape.SupportMapping(direction);
            var matrix1 = shapes[currentShape].orientation;
            result = result.Transform(matrix1);
            var value2 = shapes[currentShape].position;
            result += value2;
            return result;
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape. (Inlcuding all
        /// 'sub' shapes)
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The axis aligned bounding box of the shape.</param>
        public override JBBox GetBoundingBox(JMatrix orientation)
        {
            JBBox box;
            box.Min = mInternalBBox.Min;
            box.Max = mInternalBBox.Max;

            var localHalfExtents = 0.5f * (box.Max - box.Min);
            var localCenter = 0.5f * (box.Max + box.Min);

            var center = localCenter.Transform(orientation);

            var abs = orientation.Absolute();
            var temp = localHalfExtents.Transform(abs);

            box.Max = center + temp;
            box.Min = center - temp;
            return box;
        }

        private int currentShape;
        private List<int> currentSubShapes = new();

        /// <summary>
        /// Sets the current shape. First <see cref="CompoundShape.Prepare"/> has to be called.
        /// After SetCurrentShape the shape immitates another shape.
        /// </summary>
        /// <param name="index"></param>
        public override void SetCurrentShape(int index)
        {
            currentShape = currentSubShapes[index];
            geomCen = shapes[currentShape].Shape.SupportCenter();
            geomCen += shapes[currentShape].Position;
        }

        /// <summary>
        /// Passes a axis aligned bounding box to the shape where collision
        /// could occour.
        /// </summary>
        /// <param name="box">The bounding box where collision could occur.</param>
        /// <returns>The upper index with which <see cref="SetCurrentShape"/> can be 
        /// called.</returns>
        public override int Prepare(ref JBBox box)
        {
            currentSubShapes.Clear();

            for (var i = 0; i < shapes.Length; i++)
            {
                if (shapes[i].boundingBox.Contains(ref box) != JBBox.ContainmentType.Disjoint)
                    currentSubShapes.Add(i);
            }

            return currentSubShapes.Count;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayEnd"></param>
        /// <returns></returns>
        public override int Prepare(ref Vector3 rayOrigin, ref Vector3 rayEnd)
        {
            var box = JBBox.SmallBox;

            box.AddPoint(rayOrigin);
            box.AddPoint(rayEnd);

            return Prepare(ref box);
        }


        public override void UpdateShape()
        {
            DoShifting();
            UpdateInternalBoundingBox();
            base.UpdateShape();
        }

        protected void UpdateInternalBoundingBox()
        {
            mInternalBBox.Min = new(float.MaxValue);
            mInternalBBox.Max = new(float.MinValue);

            for (var i = 0; i < shapes.Length; i++)
            {
                shapes[i].UpdateBoundingBox();

                mInternalBBox = JBBox.CreateMerged(in mInternalBBox, in shapes[i].boundingBox);
            }
        }
    }
}
