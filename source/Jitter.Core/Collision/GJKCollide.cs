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

using Jitter.LinearMath;

namespace Jitter.Collision
{

    /// <summary>
    /// GJK based implementation of Raycasting.
    /// </summary>
    public sealed class GJKCollide
    {
        private const int MaxIterations = 15;

        private static ResourcePool<VoronoiSimplexSolver> simplexSolverPool = new ResourcePool<VoronoiSimplexSolver>();

        private static void SupportMapTransformed(ISupportMappable support, ref JMatrix orientation, ref JVector position, ref JVector direction, out JVector result)
        {
            //JVectorExtensions.Transform(ref direction, ref invOrientation, out result);
            //support.SupportMapping(ref result, out result);
            //JVectorExtensions.Transform(ref result, ref orientation, out result);
            //JVector.Add(ref result, ref position, out result);

            result.X = direction.X * orientation.M11 + direction.Y * orientation.M12 + direction.Z * orientation.M13;
            result.Y = direction.X * orientation.M21 + direction.Y * orientation.M22 + direction.Z * orientation.M23;
            result.Z = direction.X * orientation.M31 + direction.Y * orientation.M32 + direction.Z * orientation.M33;

            support.SupportMapping(ref result, out result);

            var x = result.X * orientation.M11 + result.Y * orientation.M21 + result.Z * orientation.M31;
            var y = result.X * orientation.M12 + result.Y * orientation.M22 + result.Z * orientation.M32;
            var z = result.X * orientation.M13 + result.Y * orientation.M23 + result.Z * orientation.M33;

            result.X = position.X + x;
            result.Y = position.Y + y;
            result.Z = position.Z + z;
        }

        /// <summary>
        /// Checks if given point is within a shape.
        /// </summary>
        /// <param name="support">The supportmap implementation representing the shape.</param>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="invOrientation">The inverse orientation of the shape.</param>
        /// <param name="position">The position of the shape.</param>
        /// <param name="point">The point to check.</param>
        /// <returns>Returns true if the point is within the shape, otherwise false.</returns>
        public static bool Pointcast(ISupportMappable support, ref JMatrix orientation,ref JVector position,ref JVector point)
        {
            SupportMapTransformed(support, ref orientation, ref position, ref point, out var arbitraryPoint);
            arbitraryPoint = point - arbitraryPoint;

            support.SupportCenter(out var r);
            r = JVectorExtensions.Transform(r, orientation);
            r = position + r;
            r = point - r;

            var x = point;
            float VdotR;

            var v = x - arbitraryPoint;
            var dist = v.LengthSquared();
            var epsilon = 0.0001f;

            var maxIter = MaxIterations;

            var simplexSolver = simplexSolverPool.GetNew();

            simplexSolver.Reset();

            while (dist > epsilon && maxIter-- != 0)
            {
                SupportMapTransformed(support, ref orientation, ref position, ref v, out var p);
                var w = x - p;

                var VdotW = JVector.Dot(v, w);

                if (VdotW > 0.0f)
                {
                    VdotR = JVector.Dot(v, r);

                    if (VdotR >= -(JMath.Epsilon * JMath.Epsilon)) { simplexSolverPool.GiveBack(simplexSolver); return false; }
                    else simplexSolver.Reset();
                }
                if (!simplexSolver.InSimplex(w)) simplexSolver.AddVertex(w, x, p);

                if (simplexSolver.Closest(out v)) dist = v.LengthSquared();
                else dist = 0.0f;
            }

            simplexSolverPool.GiveBack(simplexSolver);
            return true;

        }


        public static bool ClosestPoints(ISupportMappable support1, ISupportMappable support2, ref JMatrix orientation1,
            ref JMatrix orientation2, ref JVector position1, ref JVector position2,
            out JVector p1, out JVector p2, out JVector normal)
        {

            var simplexSolver = simplexSolverPool.GetNew();
            simplexSolver.Reset();

            p1 = p2 = default;

            var r = position1 - position2;
            JVector w, v;

            JVector rn,vn;

            rn = -r;

            SupportMapTransformed(support1, ref orientation1, ref position1, ref rn, out var supVertexA);

            SupportMapTransformed(support2, ref orientation2, ref position2, ref r, out var supVertexB);

            v = supVertexA - supVertexB;

            normal = default;

            var maxIter = 15;

            var distSq = v.LengthSquared();
            var epsilon = 0.00001f;

            while (distSq > epsilon && maxIter-- != 0)
            {
                vn = -v;
                SupportMapTransformed(support1, ref orientation1, ref position1, ref vn, out supVertexA);
                SupportMapTransformed(support2, ref orientation2, ref position2, ref v, out supVertexB);
                w = supVertexA - supVertexB;

                if (!simplexSolver.InSimplex(w)) simplexSolver.AddVertex(w, supVertexA, supVertexB);
                if (simplexSolver.Closest(out v))
                {
                    distSq = v.LengthSquared();
                    normal = v;
                }
                else distSq = 0.0f;
            }


            simplexSolver.ComputePoints(out p1, out p2);

            if (normal.LengthSquared() > JMath.Epsilon * JMath.Epsilon)
                normal = JVector.Normalize(normal);

            simplexSolverPool.GiveBack(simplexSolver);

            return true;

        }

        //    public static bool TimeOfImpact(ISupportMappable support1, ISupportMappable support2, ref JMatrix orientation1,
    //ref JMatrix orientation2, ref JVector position1, ref JVector position2, ref JVector sweptA, ref JVector sweptB,
    //out JVector p1, out JVector p2, out JVector normal)
    //    {

    //        VoronoiSimplexSolver simplexSolver = simplexSolverPool.GetNew();
    //        simplexSolver.Reset();

    //        float lambda = 0.0f;

    //        p1 = p2 = default;

    //        JVector x1 = position1;
    //        JVector x2 = position2;

    //        JVector r = sweptA - sweptB;
    //        JVector w, v;

    //        JVector supVertexA;
    //        JVector rn = JVector.Negate(r);
    //        SupportMapTransformed(support1, ref orientation1, ref x1, ref rn, out supVertexA);

    //        JVector supVertexB;
    //        SupportMapTransformed(support2, ref orientation2, ref x2, ref r, out supVertexB);

    //        v = supVertexA - supVertexB;

    //        bool hasResult = false;

    //        normal = default;


    //        float lastLambda = lambda;

    //        int maxIter = MaxIterations;

    //        float distSq = v.LengthSquared();
    //        float epsilon = 0.00001f;

    //        float VdotR;

    //        while ((distSq > epsilon) && (maxIter-- != 0))
    //        {

    //            JVector vn = JVector.Negate(v);
    //            SupportMapTransformed(support1, ref orientation1, ref x1, ref vn, out supVertexA);
    //            SupportMapTransformed(support2, ref orientation2, ref x2, ref v, out supVertexB);
    //            w = supVertexA - supVertexB;

    //            float VdotW = JVector.Dot(ref v, ref w);

    //            if (VdotW > 0.0f)
    //            {
    //                VdotR = JVector.Dot(ref v, ref r);

    //                if (VdotR >= -JMath.Epsilon)
    //                {
    //                    simplexSolverPool.GiveBack(simplexSolver);
    //                    return false;
    //                }
    //                else
    //                {
    //                    lambda = lambda - VdotW / VdotR;


    //                    x1 = position1 + lambda * sweptA;
    //                    x2 = position2 + lambda * sweptB;

    //                    w = supVertexA - supVertexB;

    //                    normal = v;
    //                    hasResult = true;
    //                }
    //            }
    //            if (!simplexSolver.InSimplex(w)) simplexSolver.AddVertex(w, supVertexA, supVertexB);
    //            if (simplexSolver.Closest(out v))
    //            {
    //                distSq = v.LengthSquared();
    //                normal = v;
    //                hasResult = true;
    //            }
    //            else distSq = 0.0f;
    //        }


    //        simplexSolver.ComputePoints(out p1, out p2);


    //        if (normal.LengthSquared() > JMath.Epsilon * JMath.Epsilon)
    //            normal.Normalize();

    //        p1 = p1 - lambda * sweptA;
    //        p2 = p2 - lambda * sweptB;

    //        simplexSolverPool.GiveBack(simplexSolver);

    //        return true;

    //    }

    // see: btSubSimplexConvexCast.cpp

        /// <summary>
        /// Checks if a ray definied through it's origin and direction collides
        /// with a shape.
        /// </summary>
        /// <param name="support">The supportmap implementation representing the shape.</param>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="invOrientation">The inverse orientation of the shape.</param>
        /// <param name="position">The position of the shape.</param>
        /// <param name="origin">The origin of the ray.</param>
        /// <param name="direction">The direction of the ray.</param>
        /// <param name="fraction">The fraction which gives information where at the 
        /// ray the collision occured. The hitPoint is calculated by: origin+friction*direction.</param>
        /// <param name="normal">The normal from the ray collision.</param>
        /// <returns>Returns true if the ray hit the shape, false otherwise.</returns>
        public static bool Raycast(ISupportMappable support, ref JMatrix orientation, ref JMatrix invOrientation,
            ref JVector position,ref JVector origin,ref JVector direction, out float fraction, out JVector normal)
        {
            var simplexSolver = simplexSolverPool.GetNew();
            simplexSolver.Reset();

            normal = default;
            fraction = float.MaxValue;

            var lambda = 0.0f;

            var r = direction;
            var x = origin;

            SupportMapTransformed(support, ref orientation, ref position, ref r, out var arbitraryPoint);
            var v = x - arbitraryPoint;

            var maxIter = MaxIterations;

            var distSq = v.LengthSquared();
            var epsilon = 0.000001f;

            float VdotR;

            while (distSq > epsilon && maxIter-- != 0)
            {
                SupportMapTransformed(support, ref orientation, ref position, ref v, out var p);
                var w = x - p;

                var VdotW = JVector.Dot(v, w);

                if (VdotW > 0.0f)
                {
                    VdotR = JVector.Dot(v, r);

                    if (VdotR >= -JMath.Epsilon)
                    {
                        simplexSolverPool.GiveBack(simplexSolver);
                        return false;
                    }
                    else
                    {
                        lambda = lambda - VdotW / VdotR;
                        x = r * lambda;
                        x = origin + x;
                        w = x - p;
                        normal = v;
                    }
                }
                if (!simplexSolver.InSimplex(w)) simplexSolver.AddVertex(w, x, p);
                if (simplexSolver.Closest(out v)) { distSq = v.LengthSquared();  }
                else distSq = 0.0f;
            }

            // Giving back the fraction like this *should* work
            // but is inaccurate against large objects:
            // fraction = lambda;

            JVector p1;
            simplexSolver.ComputePoints(out p1, out var p2);

            p2 = p2 - origin;
            fraction = p2.Length() / direction.Length();

            if (normal.LengthSquared() > JMath.Epsilon * JMath.Epsilon)
                normal = JVector.Normalize(normal);

            simplexSolverPool.GiveBack(simplexSolver);

            return true;
        }

        // see: btVoronoiSimplexSolver.cpp

        // Bullet has problems with raycasting large objects - so does jitter
        // hope to fix that in the next versions.

        /*
          Bullet for XNA Copyright (c) 2003-2007 Vsevolod Klementjev http://www.codeplex.com/xnadevru
          Bullet original C++ version Copyright (c) 2003-2007 Erwin Coumans http://bulletphysics.com

          This software is provided 'as-is', without any express or implied
          warranty.  In no event will the authors be held liable for any damages
          arising from the use of this software.

          Permission is granted to anyone to use this software for any purpose,
          including commercial applications, and to alter it and redistribute it
          freely, subject to the following restrictions:

          1. The origin of this software must not be misrepresented; you must not
             claim that you wrote the original software. If you use this software
             in a product, an acknowledgment in the product documentation would be
             appreciated but is not required.
          2. Altered source versions must be plainly marked as such, and must not be
             misrepresented as being the original software.
          3. This notice may not be removed or altered from any source distribution.
        */

        private class UsageBitfield
        {
            private bool _usedVertexA, _usedVertexB, _usedVertexC, _usedVertexD;

            public bool UsedVertexA { get => _usedVertexA;
                set => _usedVertexA = value;
            }
            public bool UsedVertexB { get => _usedVertexB;
                set => _usedVertexB = value;
            }
            public bool UsedVertexC { get => _usedVertexC;
                set => _usedVertexC = value;
            }
            public bool UsedVertexD { get => _usedVertexD;
                set => _usedVertexD = value;
            }

            public void Reset()
            {
                _usedVertexA = _usedVertexB = _usedVertexC = _usedVertexD = false;
            }
        }

        private class SubSimplexClosestResult
        {
            private JVector _closestPointOnSimplex;

            //MASK for m_usedVertices
            //stores the simplex vertex-usage, using the MASK, 
            // if m_usedVertices & MASK then the related vertex is used
            private UsageBitfield _usedVertices = new UsageBitfield();
            private float[] _barycentricCoords = new float[4];
            private bool _degenerate;

            public JVector ClosestPointOnSimplex { get => _closestPointOnSimplex;
                set => _closestPointOnSimplex = value;
            }
            public UsageBitfield UsedVertices { get => _usedVertices;
                set => _usedVertices = value;
            }
            public float[] BarycentricCoords { get => _barycentricCoords;
                set => _barycentricCoords = value;
            }
            public bool Degenerate { get => _degenerate;
                set => _degenerate = value;
            }

            public void Reset()
            {
                _degenerate = false;
                SetBarycentricCoordinates();
                _usedVertices.Reset();
            }

            public bool IsValid =>
                _barycentricCoords[0] >= 0f &&
                _barycentricCoords[1] >= 0f &&
                _barycentricCoords[2] >= 0f &&
                _barycentricCoords[3] >= 0f;

            public void SetBarycentricCoordinates()
            {
                SetBarycentricCoordinates(0f, 0f, 0f, 0f);
            }

            public void SetBarycentricCoordinates(float a, float b, float c, float d)
            {
                _barycentricCoords[0] = a;
                _barycentricCoords[1] = b;
                _barycentricCoords[2] = c;
                _barycentricCoords[3] = d;
            }
        }

        /// VoronoiSimplexSolver is an implementation of the closest point distance
        /// algorithm from a 1-4 points simplex to the origin.
        /// Can be used with GJK, as an alternative to Johnson distance algorithm. 
        private class VoronoiSimplexSolver
        {
            private const int VertexA = 0, VertexB = 1, VertexC = 2, VertexD = 3;

            private const int VoronoiSimplexMaxVerts = 5;
            private const bool CatchDegenerateTetrahedron = true;

            private int _numVertices;

            private JVector[] _simplexVectorW = new JVector[VoronoiSimplexMaxVerts];
            private JVector[] _simplexPointsP = new JVector[VoronoiSimplexMaxVerts];
            private JVector[] _simplexPointsQ = new JVector[VoronoiSimplexMaxVerts];

            private JVector _cachedPA;
            private JVector _cachedPB;
            private JVector _cachedV;
            private JVector _lastW;
            private bool _cachedValidClosest;

            private SubSimplexClosestResult _cachedBC = new SubSimplexClosestResult();

            // Note that this assumes ray-casts and point-casts will always be called from the
            // same thread which I assume is true from the _cachedBC member
            // If this needs to made multi-threaded a resource pool will be needed
            private SubSimplexClosestResult tempResult = new SubSimplexClosestResult();

            private bool _needsUpdate;

            public bool FullSimplex => _numVertices == 4;

            public int NumVertices => _numVertices;

            public void Reset()
            {
                _cachedValidClosest = false;
                _numVertices = 0;
                _needsUpdate = true;
                _lastW = new JVector(1e30f, 1e30f, 1e30f);
                _cachedBC.Reset();
            }

            public void AddVertex(JVector w, JVector p, JVector q)
            {
                _lastW = w;
                _needsUpdate = true;

                _simplexVectorW[_numVertices] = w;
                _simplexPointsP[_numVertices] = p;
                _simplexPointsQ[_numVertices] = q;

                _numVertices++;
            }

            //return/calculate the closest vertex
            public bool Closest(out JVector v)
            {
                var succes = UpdateClosestVectorAndPoints();
                v = _cachedV;
                return succes;
            }

            public float MaxVertex
            {
                get
                {
                    var numverts = NumVertices;
                    float maxV = 0f, curLen2;
                    for (var i = 0; i < numverts; i++)
                    {
                        curLen2 = _simplexVectorW[i].LengthSquared();
                        if (maxV < curLen2) maxV = curLen2;
                    }
                    return maxV;
                }
            }

            //return the current simplex
            public int GetSimplex(out JVector[] pBuf, out JVector[] qBuf, out JVector[] yBuf)
            {
                var numverts = NumVertices;
                pBuf = new JVector[numverts];
                qBuf = new JVector[numverts];
                yBuf = new JVector[numverts];
                for (var i = 0; i < numverts; i++)
                {
                    yBuf[i] = _simplexVectorW[i];
                    pBuf[i] = _simplexPointsP[i];
                    qBuf[i] = _simplexPointsQ[i];
                }
                return numverts;
            }

            public bool InSimplex(JVector w)
            {
                //check in case lastW is already removed
                if (w == _lastW) return true;

                //w is in the current (reduced) simplex
                var numverts = NumVertices;
                for (var i = 0; i < numverts; i++)
                    if (_simplexVectorW[i] == w) return true;

                return false;
            }

            public void BackupClosest(out JVector v)
            {
                v = _cachedV;
            }

            public bool EmptySimplex => NumVertices == 0;

            public void ComputePoints(out JVector p1, out JVector p2)
            {
                UpdateClosestVectorAndPoints();
                p1 = _cachedPA;
                p2 = _cachedPB;
            }

            public void RemoveVertex(int index)
            {
                 _numVertices--;
                _simplexVectorW[index] = _simplexVectorW[_numVertices];
                _simplexPointsP[index] = _simplexPointsP[_numVertices];
                _simplexPointsQ[index] = _simplexPointsQ[_numVertices];
            }

            public void ReduceVertices(UsageBitfield usedVerts)
            {
                if (NumVertices >= 4 && !usedVerts.UsedVertexD) RemoveVertex(3);
                if (NumVertices >= 3 && !usedVerts.UsedVertexC) RemoveVertex(2);
                if (NumVertices >= 2 && !usedVerts.UsedVertexB) RemoveVertex(1);
                if (NumVertices >= 1 && !usedVerts.UsedVertexA) RemoveVertex(0);
            }

            public bool UpdateClosestVectorAndPoints()
            {
                if (_needsUpdate)
                {
                    _cachedBC.Reset();
                    _needsUpdate = false;

                    JVector p, a, b, c, d;
                    switch (NumVertices)
                    {
                        case 0:
                            _cachedValidClosest = false;
                            break;
                        case 1:
                            _cachedPA = _simplexPointsP[0];
                            _cachedPB = _simplexPointsQ[0];
                            _cachedV = _cachedPA - _cachedPB;
                            _cachedBC.Reset();
                            _cachedBC.SetBarycentricCoordinates(1f, 0f, 0f, 0f);
                            _cachedValidClosest = _cachedBC.IsValid;
                            break;
                        case 2:
                            //closest point origin from line segment
                            var from = _simplexVectorW[0];
                            var to = _simplexVectorW[1];
                            JVector nearest;

                            var diff = from * -1;
                            var v = to - from;
                            var t = JVector.Dot(v, diff);

                            if (t > 0)
                            {
                                var dotVV = v.LengthSquared();
                                if (t < dotVV)
                                {
                                    t /= dotVV;
                                    diff -= t * v;
                                    _cachedBC.UsedVertices.UsedVertexA = true;
                                    _cachedBC.UsedVertices.UsedVertexB = true;
                                }
                                else
                                {
                                    t = 1;
                                    diff -= v;
                                    //reduce to 1 point
                                    _cachedBC.UsedVertices.UsedVertexB = true;
                                }
                            }
                            else
                            {
                                t = 0;
                                //reduce to 1 point
                                _cachedBC.UsedVertices.UsedVertexA = true;
                            }

                            _cachedBC.SetBarycentricCoordinates(1 - t, t, 0, 0);
                            nearest = from + t * v;

                            _cachedPA = _simplexPointsP[0] + t * (_simplexPointsP[1] - _simplexPointsP[0]);
                            _cachedPB = _simplexPointsQ[0] + t * (_simplexPointsQ[1] - _simplexPointsQ[0]);
                            _cachedV = _cachedPA - _cachedPB;

                            ReduceVertices(_cachedBC.UsedVertices);

                            _cachedValidClosest = _cachedBC.IsValid;
                            break;
                        case 3:
                            //closest point origin from triangle
                            p = new JVector();
                            a = _simplexVectorW[0];
                            b = _simplexVectorW[1];
                            c = _simplexVectorW[2];

                            ClosestPtPointTriangle(p, a, b, c, ref _cachedBC);
                            _cachedPA = _simplexPointsP[0] * _cachedBC.BarycentricCoords[0] +
                                            _simplexPointsP[1] * _cachedBC.BarycentricCoords[1] +
                                            _simplexPointsP[2] * _cachedBC.BarycentricCoords[2] +
                                            _simplexPointsP[3] * _cachedBC.BarycentricCoords[3];

                            _cachedPB = _simplexPointsQ[0] * _cachedBC.BarycentricCoords[0] +
                                            _simplexPointsQ[1] * _cachedBC.BarycentricCoords[1] +
                                            _simplexPointsQ[2] * _cachedBC.BarycentricCoords[2]+
                                            _simplexPointsQ[3] * _cachedBC.BarycentricCoords[3];

                            _cachedV = _cachedPA - _cachedPB;

                            ReduceVertices(_cachedBC.UsedVertices);
                            _cachedValidClosest = _cachedBC.IsValid;
                            break;
                        case 4:
                            p = new JVector();
                            a = _simplexVectorW[0];
                            b = _simplexVectorW[1];
                            c = _simplexVectorW[2];
                            d = _simplexVectorW[3];

                            var hasSeperation = ClosestPtPointTetrahedron(p, a, b, c, d, ref _cachedBC);

                            if (hasSeperation)
                            {
                                _cachedPA = _simplexPointsP[0] * _cachedBC.BarycentricCoords[0] +
                                                _simplexPointsP[1] * _cachedBC.BarycentricCoords[1] +
                                                _simplexPointsP[2] * _cachedBC.BarycentricCoords[2] +
                                                _simplexPointsP[3] * _cachedBC.BarycentricCoords[3];

                                _cachedPB = _simplexPointsQ[0] * _cachedBC.BarycentricCoords[0] +
                                                _simplexPointsQ[1] * _cachedBC.BarycentricCoords[1] +
                                                _simplexPointsQ[2] * _cachedBC.BarycentricCoords[2] +
                                                _simplexPointsQ[3] * _cachedBC.BarycentricCoords[3];

                                _cachedV = _cachedPA - _cachedPB;
                                ReduceVertices(_cachedBC.UsedVertices);
                            }
                            else
                            {
                                if (_cachedBC.Degenerate)
                                {
                                    _cachedValidClosest = false;
                                }
                                else
                                {
                                    _cachedValidClosest = true;
                                    //degenerate case == false, penetration = true + zero
                                    _cachedV.X = _cachedV.Y = _cachedV.Z = 0f;
                                }
                                break; // !!!!!!!!!!!! proverit na vsakiy sluchai
                            }

                            _cachedValidClosest = _cachedBC.IsValid;

                            //closest point origin from tetrahedron
                            break;
                        default:
                            _cachedValidClosest = false;
                            break;
                    }
                }

                return _cachedValidClosest;
            }

            public bool ClosestPtPointTriangle(JVector p, JVector a, JVector b, JVector c,
                ref SubSimplexClosestResult result)
            {
                result.UsedVertices.Reset();

                float v, w;

                // Check if P in vertex region outside A
                var ab = b - a;
                var ac = c - a;
                var ap = p - a;
                var d1 = JVector.Dot(ab, ap);
                var d2 = JVector.Dot(ac, ap);
                if (d1 <= 0f && d2 <= 0f)
                {
                    result.ClosestPointOnSimplex = a;
                    result.UsedVertices.UsedVertexA = true;
                    result.SetBarycentricCoordinates(1, 0, 0, 0);
                    return true; // a; // barycentric coordinates (1,0,0)
                }

                // Check if P in vertex region outside B
                var bp = p - b;
                var d3 = JVector.Dot(ab, bp);
                var d4 = JVector.Dot(ac, bp);
                if (d3 >= 0f && d4 <= d3)
                {
                    result.ClosestPointOnSimplex = b;
                    result.UsedVertices.UsedVertexB = true;
                    result.SetBarycentricCoordinates(0, 1, 0, 0);

                    return true; // b; // barycentric coordinates (0,1,0)
                }
                // Check if P in edge region of AB, if so return projection of P onto AB
                var vc = d1 * d4 - d3 * d2;
                if (vc <= 0f && d1 >= 0f && d3 <= 0f)
                {
                    v = d1 / (d1 - d3);
                    result.ClosestPointOnSimplex = a + v * ab;
                    result.UsedVertices.UsedVertexA = true;
                    result.UsedVertices.UsedVertexB = true;
                    result.SetBarycentricCoordinates(1 - v, v, 0, 0);
                    return true;
                    //return a + v * ab; // barycentric coordinates (1-v,v,0)
                }

                // Check if P in vertex region outside C
                var cp = p - c;
                var d5 = JVector.Dot(ab, cp);
                var d6 = JVector.Dot(ac, cp);
                if (d6 >= 0f && d5 <= d6)
                {
                    result.ClosestPointOnSimplex = c;
                    result.UsedVertices.UsedVertexC = true;
                    result.SetBarycentricCoordinates(0, 0, 1, 0);
                    return true;//c; // barycentric coordinates (0,0,1)
                }

                // Check if P in edge region of AC, if so return projection of P onto AC
                var vb = d5 * d2 - d1 * d6;
                if (vb <= 0f && d2 >= 0f && d6 <= 0f)
                {
                    w = d2 / (d2 - d6);
                    result.ClosestPointOnSimplex = a + w * ac;
                    result.UsedVertices.UsedVertexA = true;
                    result.UsedVertices.UsedVertexC = true;
                    result.SetBarycentricCoordinates(1 - w, 0, w, 0);
                    return true;
                    //return a + w * ac; // barycentric coordinates (1-w,0,w)
                }

                // Check if P in edge region of BC, if so return projection of P onto BC
                var va = d3 * d6 - d5 * d4;
                if (va <= 0f && d4 - d3 >= 0f && d5 - d6 >= 0f)
                {
                    w = (d4 - d3) / (d4 - d3 + (d5 - d6));

                    result.ClosestPointOnSimplex = b + w * (c - b);
                    result.UsedVertices.UsedVertexB = true;
                    result.UsedVertices.UsedVertexC = true;
                    result.SetBarycentricCoordinates(0, 1 - w, w, 0);
                    return true;
                    // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
                }

                // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
                var denom = 1.0f / (va + vb + vc);
                v = vb * denom;
                w = vc * denom;

                result.ClosestPointOnSimplex = a + ab * v + ac * w;
                result.UsedVertices.UsedVertexA = true;
                result.UsedVertices.UsedVertexB = true;
                result.UsedVertices.UsedVertexC = true;
                result.SetBarycentricCoordinates(1 - v - w, v, w, 0);

                return true;
            }

            /// Test if point p and d lie on opposite sides of plane through abc
            public int PointOutsideOfPlane(JVector p, JVector a, JVector b, JVector c, JVector d)
            {
                var normal = JVector.Cross(b - a, c - a);

                var signp = JVector.Dot(p - a, normal); // [AP AB AC]
                var signd = JVector.Dot(d - a, normal); // [AD AB AC]

                //if (CatchDegenerateTetrahedron)
                    if (signd * signd < 1e-4f * 1e-4f) return -1;

                // Points on opposite sides if expression signs are opposite
                return signp * signd < 0f ? 1 : 0;
            }

            public bool ClosestPtPointTetrahedron(JVector p, JVector a, JVector b, JVector c, JVector d,
                ref SubSimplexClosestResult finalResult)
            {
                tempResult.Reset();

                // Start out assuming point inside all halfspaces, so closest to itself
                finalResult.ClosestPointOnSimplex = p;
                finalResult.UsedVertices.Reset();
                finalResult.UsedVertices.UsedVertexA = true;
                finalResult.UsedVertices.UsedVertexB = true;
                finalResult.UsedVertices.UsedVertexC = true;
                finalResult.UsedVertices.UsedVertexD = true;

                var pointOutsideABC = PointOutsideOfPlane(p, a, b, c, d);
                var pointOutsideACD = PointOutsideOfPlane(p, a, c, d, b);
                var pointOutsideADB = PointOutsideOfPlane(p, a, d, b, c);
                var pointOutsideBDC = PointOutsideOfPlane(p, b, d, c, a);

                if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0)
                {
                    finalResult.Degenerate = true;
                    return false;
                }

                if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0)
                    return false;

                var bestSqDist = float.MaxValue;
                // If point outside face abc then compute closest point on abc
                if (pointOutsideABC != 0)
                {
                    ClosestPtPointTriangle(p, a, b, c, ref tempResult);
                    var q = tempResult.ClosestPointOnSimplex;

                    var sqDist = (q - p).LengthSquared();
                    // Update best closest point if (squared) distance is less than current best
                    if (sqDist < bestSqDist)
                    {
                        bestSqDist = sqDist;
                        finalResult.ClosestPointOnSimplex = q;
                        //convert result bitmask!
                        finalResult.UsedVertices.Reset();
                        finalResult.UsedVertices.UsedVertexA = tempResult.UsedVertices.UsedVertexA;
                        finalResult.UsedVertices.UsedVertexB = tempResult.UsedVertices.UsedVertexB;
                        finalResult.UsedVertices.UsedVertexC = tempResult.UsedVertices.UsedVertexC;
                        finalResult.SetBarycentricCoordinates(
                                tempResult.BarycentricCoords[VertexA],
                                tempResult.BarycentricCoords[VertexB],
                                tempResult.BarycentricCoords[VertexC],
                                0);
                    }
                }

                // Repeat test for face acd
                if (pointOutsideACD != 0)
                {
                    ClosestPtPointTriangle(p, a, c, d, ref tempResult);
                    var q = tempResult.ClosestPointOnSimplex;
                    //convert result bitmask!

                    var sqDist = (q - p).LengthSquared();
                    if (sqDist < bestSqDist)
                    {
                        bestSqDist = sqDist;
                        finalResult.ClosestPointOnSimplex = q;
                        finalResult.UsedVertices.Reset();
                        finalResult.UsedVertices.UsedVertexA = tempResult.UsedVertices.UsedVertexA;
                        finalResult.UsedVertices.UsedVertexC = tempResult.UsedVertices.UsedVertexB;
                        finalResult.UsedVertices.UsedVertexD = tempResult.UsedVertices.UsedVertexC;
                        finalResult.SetBarycentricCoordinates(
                                tempResult.BarycentricCoords[VertexA],
                                0,
                                tempResult.BarycentricCoords[VertexB],
                                tempResult.BarycentricCoords[VertexC]);
                    }
                }
                // Repeat test for face adb

                if (pointOutsideADB != 0)
                {
                    ClosestPtPointTriangle(p, a, d, b, ref tempResult);
                    var q = tempResult.ClosestPointOnSimplex;
                    //convert result bitmask!

                    var sqDist = (q - p).LengthSquared();
                    if (sqDist < bestSqDist)
                    {
                        bestSqDist = sqDist;
                        finalResult.ClosestPointOnSimplex = q;
                        finalResult.UsedVertices.Reset();
                        finalResult.UsedVertices.UsedVertexA = tempResult.UsedVertices.UsedVertexA;
                        finalResult.UsedVertices.UsedVertexD = tempResult.UsedVertices.UsedVertexB;
                        finalResult.UsedVertices.UsedVertexB = tempResult.UsedVertices.UsedVertexC;
                        finalResult.SetBarycentricCoordinates(
                                tempResult.BarycentricCoords[VertexA],
                                tempResult.BarycentricCoords[VertexC],
                                0,
                                tempResult.BarycentricCoords[VertexB]);

                    }
                }
                // Repeat test for face bdc

                if (pointOutsideBDC != 0)
                {
                    ClosestPtPointTriangle(p, b, d, c, ref tempResult);
                    var q = tempResult.ClosestPointOnSimplex;
                    //convert result bitmask!
                    var sqDist = (q - p).LengthSquared();
                    if (sqDist < bestSqDist)
                    {
                        bestSqDist = sqDist;
                        finalResult.ClosestPointOnSimplex = q;
                        finalResult.UsedVertices.Reset();
                        finalResult.UsedVertices.UsedVertexB = tempResult.UsedVertices.UsedVertexA;
                        finalResult.UsedVertices.UsedVertexD = tempResult.UsedVertices.UsedVertexB;
                        finalResult.UsedVertices.UsedVertexC = tempResult.UsedVertices.UsedVertexC;

                        finalResult.SetBarycentricCoordinates(
                                0,
                                tempResult.BarycentricCoords[VertexA],
                                tempResult.BarycentricCoords[VertexC],
                                tempResult.BarycentricCoords[VertexB]);
                    }
                }

                //help! we ended up full !

                if (finalResult.UsedVertices.UsedVertexA &&
                    finalResult.UsedVertices.UsedVertexB &&
                    finalResult.UsedVertices.UsedVertexC &&
                    finalResult.UsedVertices.UsedVertexD)
                {
                    return true;
                }

                return true;
            }
        }
    }
}
