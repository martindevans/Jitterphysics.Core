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
using System.Numerics;
using Jitter.LinearMath;

namespace Jitter.Collision.Shapes
{

    /// <summary>
    /// A <see cref="BaseShape"/> representing a cone.
    /// </summary>
    public class ConeShape : BaseShape
    {
        private float _height;
        private float _radius;

        /// <summary>
        /// The height of the cone.
        /// </summary>
        public float Height
        {
            get => _height;
            set
            {
                _height = value;
                UpdateShape();
            }
        }

        /// <summary>
        /// The radius of the cone base.
        /// </summary>
        public float Radius
        {
            get => _radius;
            set
            {
                _radius = value;
                UpdateShape();
            }
        }

        /// <summary>
        /// Initializes a new instance of the ConeShape class.
        /// </summary>
        /// <param name="height">The height of the cone.</param>
        /// <param name="radius">The radius of the cone base.</param>
        public ConeShape(float height, float radius)
        {
            this._height = height;
            this._radius = radius;

            UpdateShape();
        }

        public override void UpdateShape()
        {
            sina = _radius / (float)Math.Sqrt(_radius * _radius + _height * _height);
            base.UpdateShape();
        }

        private float sina;

        /// <summary>
        /// 
        /// </summary>
        public override void CalculateMassInertia()
        {
            Mass = 1.0f / 3.0f * MathF.PI * _radius * _radius * _height;

            // inertia through center of mass axis.
            var i = JMatrix.Identity;
            i.M11 = 3.0f / 80.0f * Mass * (_radius * _radius + 4 * _height * _height);
            i.M22 = 3.0f / 10.0f * Mass * _radius * _radius;
            i.M33 = 3.0f / 80.0f * Mass * (_radius * _radius + 4 * _height * _height);
            Inertia = i;

            // J_x=J_y=3/20 M (R^2+4 H^2)

            // the supportmap center is in the half height, the real geomcenter is:
            geomCen = default;
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
            Vector3 result;
            var sigma = (float)Math.Sqrt(direction.X * direction.X + direction.Z * direction.Z);

            if (direction.Y > direction.Length() * sina)
            {
                result.X = 0.0f;
                result.Y = 2.0f / 3.0f * _height;
                result.Z = 0.0f;
            }
            else if (sigma > 0.0f)
            {
                result.X = _radius * direction.X / sigma;
                result.Y = -(1.0f / 3.0f) * _height;
                result.Z = _radius * direction.Z / sigma;
            }
            else
            {
                result.X = 0.0f;
                result.Y = -(1.0f / 3.0f) * _height;
                result.Z = 0.0f;
            }

            return result;
        }
    }
}
