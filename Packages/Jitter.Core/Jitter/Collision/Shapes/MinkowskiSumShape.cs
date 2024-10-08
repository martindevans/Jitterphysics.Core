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
using System.Numerics;

namespace Jitter.Collision.Shapes
{
    public class MinkowskiSumShape : BaseShape
    {
        private Vector3 shifted;
        private List<BaseShape> shapes = new();

        public MinkowskiSumShape(IEnumerable<BaseShape> shapes)
        {
            AddShapes(shapes);
        }

        public void AddShapes(IEnumerable<BaseShape> shapes)
        {
            foreach (var shape in shapes)
            {
                if (shape is Multishape) throw new("Multishapes not supported by MinkowskiSumShape.");
                this.shapes.Add(shape);
            }

            UpdateShape();
        }

        public void AddShape(BaseShape shape)
        {
            if (shape is Multishape) throw new("Multishapes not supported by MinkowskiSumShape.");
            shapes.Add(shape);

            UpdateShape();
        }

        public bool Remove(BaseShape shape)
        {
            if (shapes.Count == 1) throw new("There must be at least one shape.");
            var result = shapes.Remove(shape);
            UpdateShape();
            return result;
        }

        public Vector3 Shift()
        {
            return -1 * shifted;
        }

        public override void CalculateMassInertia()
        {
            (Mass, Inertia) = BaseShape.CalculateMassInertia(this, out shifted);
        }

        public override Vector3 SupportMapping(Vector3 direction)
        {
            Vector3 temp2 = default;

            for (var i = 0; i < shapes.Count; i++)
            {
                var temp1 = shapes[i].SupportMapping(direction);
                temp2 += temp1;
            }

            var result = temp2 - shifted;
            return result;
        }

    }
}
