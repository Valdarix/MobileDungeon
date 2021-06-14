using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;

namespace UnityEngine.U2D.Common.UTess
{
    enum UEventType
    {
        EVENT_POINT = 0,
        EVENT_END = 1,
        EVENT_START = 2,
    };

    struct UEvent
    {
        public float2 a;
        public float2 b;
        public int idx;
        public int type;
    };

    struct UHull
    {
        public float2 a;
        public float2 b;
        public int idx;

        public ArraySlice<int> ilarray;
        public int ilcount;
        public ArraySlice<int> iuarray;
        public int iucount;
    };

    struct UStar
    {
        public ArraySlice<int> points;
        public int pointCount;
    };

    struct UBounds
    {
        public double2 min;
        public double2 max;
    };

    struct UCircle
    {
        public float2 center;
        public float radius;
    };

    struct UTriangle
    {
        public float2 va;
        public float2 vb;
        public float2 vc;
        public UCircle c;
        public float area;
        public int3 indices;
    };

    struct UEncroachingSegment
    {
        public float2 a;
        public float2 b;
        public int index;
    }

    internal interface ICondition2<in T, in U>
    {
        bool Test(T x, U y, ref float t);
    }

    struct XCompare : IComparer<double>
    {
        public int Compare(double a, double b)
        {
            return (a < b) ? -1 : 1;
        }
    }

    unsafe struct IntersectionCompare : IComparer<int2>
    {
        public NativeArray<double2> points;
        public NativeArray<int2> edges;

        public fixed double xvasort[4];
        public fixed double xvbsort[4];

        public int Compare(int2 a, int2 b)
        {
            var e1a = edges[a.x];
            var e1b = edges[a.y];
            var e2a = edges[b.x];
            var e2b = edges[b.y];

            xvasort[0] = points[e1a.x].x;
            xvasort[1] = points[e1a.y].x;
            xvasort[2] = points[e1b.x].x;
            xvasort[3] = points[e1b.y].x;

            xvbsort[0] = points[e2a.x].x;
            xvbsort[1] = points[e2a.y].x;
            xvbsort[2] = points[e2b.x].x;
            xvbsort[3] = points[e2b.y].x;

            fixed (double* xvasortPtr = xvasort)
            {
                UTess.InsertionSort<double, XCompare>(xvasortPtr, 0, 3, new XCompare());
            }

            fixed (double* xvbsortPtr = xvbsort)
            {
                UTess.InsertionSort<double, XCompare>(xvbsortPtr, 0, 3, new XCompare());
            }

            for (int i = 0; i < 4; ++i)
                if (xvasort[i] - xvbsort[i] != 0)
                    return xvasort[i] < xvbsort[i] ? -1 : 1;
            return points[e1a.x].y < points[e1a.x].y ? -1 : 1;
        }
    }

    struct TessEventCompare : IComparer<UEvent>
    {
        public int Compare(UEvent a, UEvent b)
        {
            float f = (a.a.x - b.a.x);
            if (0 != f)
                return (f > 0) ? 1 : -1;

            f = (a.a.y - b.a.y);
            if (0 != f)
                return (f > 0) ? 1 : -1;

            int i = a.type - b.type;
            if (0 != i)
                return i;

            if (a.type != (int)UEventType.EVENT_POINT)
            {
                float o = UTess.OrientFast(a.a, a.b, b.b);
                if (0 != o)
                {
                    return (o > 0) ? 1 : -1;
                }
            }

            return a.idx - b.idx;
        }
    }

    struct TessEdgeCompare : IComparer<int2>
    {
        public int Compare(int2 a, int2 b)
        {
            int i = a.x - b.x;
            if (0 != i)
                return i;
            i = a.y - b.y;
            return i;
        }
    }

    struct TessCellCompare : IComparer<int3>
    {
        public int Compare(int3 a, int3 b)
        {
            int i = a.x - b.x;
            if (0 != i)
                return i;
            i = a.y - b.y;
            if (0 != i)
                return i;
            i = a.z - b.z;
            return i;
        }
    }

    struct TessJunctionCompare : IComparer<int2>
    {
        public int Compare(int2 a, int2 b)
        {
            int i = a.x - b.x;
            if (0 != i)
                return i;
            i = a.y - b.y;
            return i;
        }
    }

    struct DelaEdgeCompare : IComparer<int4>
    {
        public int Compare(int4 a, int4 b)
        {
            int i = a.x - b.x;
            if (0 != i)
                return i;
            i = a.y - b.y;
            if (0 != i)
                return i;
            i = a.z - b.z;
            if (0 != i)
                return i;
            i = a.w - b.w;
            return i;
        }
    }

    struct TessLink
    {

        internal NativeArray<int> roots;
        internal NativeArray<int> ranks;

        internal static TessLink CreateLink(int count)
        {
            TessLink link = new TessLink();
            link.roots = new NativeArray<int>(count, Allocator.Temp);
            link.ranks = new NativeArray<int>(count, Allocator.Temp);

            for (int i = 0; i < count; ++i)
            {
                link.roots[i] = i;
                link.ranks[i] = 0;
            }
            return link;
        }

        internal static void DestroyLink(TessLink link)
        {
            link.ranks.Dispose();
            link.roots.Dispose();
        }

        internal int Find(int x)
        {
            var x0 = x;
            while (roots[x] != x)
            {
                x = roots[x];
            }
            while (roots[x0] != x)
            {
                var y = roots[x0];
                roots[x0] = x;
                x0 = y;
            }
            return x;
        }

        internal void Link(int x, int y)
        {
            var xr = Find(x);
            var yr = Find(y);
            if (xr == yr)
            {
                return;
            }
            var xd = ranks[xr];
            var yd = ranks[yr];
            if (xd < yd)
            {
                roots[xr] = yr;
            }
            else if (yd < xd)
            {
                roots[yr] = xr;
            }
            else
            {
                roots[yr] = xr;
                ++ranks[xr];
            }
        }
    };

    internal struct UTess
    {
    
        // Max Edge Count with Subdivision allowed. This is already a very relaxed limit
        // and anything beyond are basically littered with numerous paths.
        internal static readonly  int kMaxArea = 65536;
        internal static readonly  int kMaxEdgeCount = 65536;
        internal static readonly  int kMaxIndexCount = 65536;
        internal static readonly  int kMaxVertexCount = 65536;
        internal static readonly  int kMaxTriangleCount = kMaxIndexCount / 3;
        internal static readonly  int kMaxSmoothenIterations = 124;
    
        // Search Lower Bounds 
        internal static int GetLower<T, U, X>(NativeArray<T> values, int count, U check, X condition)
            where T : struct where U : struct where X : ICondition2<T, U>
        {
            int l = 0;
            int h = count - 1;
            int i = l - 1;
            while (l <= h)
            {
                int m = ((int)(l + h)) >> 1;
                float t = 0;
                if (condition.Test(values[m], check, ref t))
                {
                    i = m;
                    l = m + 1;
                }
                else
                {
                    h = m - 1;
                }
            }
            return i;
        }

        // Search Upper Bounds
        internal static int GetUpper<T, U, X>(NativeArray<T> values, int count, U check, X condition)
            where T : struct where U : struct where X : ICondition2<T, U>
        {
            int l = 0;
            int h = count - 1;
            int i = h + 1;
            while (l <= h)
            {
                int m = ((int)(l + h)) >> 1;
                float t = 0;
                if (condition.Test(values[m], check, ref t))
                {
                    i = m;
                    h = m - 1;
                }
                else
                {
                    l = m + 1;
                }
            }
            return i;
        }

        // Search for Equal
        internal static int GetEqual<T, U, X>(NativeArray<T> values, int count, U check, X condition)
            where T : struct where U : struct where X : ICondition2<T, U>
        {
            int l = 0;
            int h = count - 1;
            while (l <= h)
            {
                int m = ((int)(l + h)) >> 1;
                float t = 0;
                condition.Test(values[m], check, ref t);
                if (t == 0)
                {
                    return m;
                }
                else if (t <= 0)
                {
                    l = m + 1;
                }
                else
                {
                    h = m - 1;
                }
            }
            return -1;
        }

        internal static void BuildTrianglesAndEdges(NativeArray<float2> vertices, int vertexCount, NativeArray<int> indices, int indexCount, ref NativeArray<UTriangle> triangles, ref int triangleCount, ref NativeArray<int4> delaEdges, ref int delaEdgeCount, ref float maxArea)
        {
            // Check if there are invalid triangles or segments.
            for (int i = 0; i < indexCount; i += 3)
            {
                UTriangle tri = new UTriangle();
                var i0 = indices[i + 0];
                var i1 = indices[i + 1];
                var i2 = indices[i + 2];
                tri.va = vertices[i0];
                tri.vb = vertices[i1];
                tri.vc = vertices[i2];
                tri.c = UTess.CircumCircle(tri);
                tri.area = UTess.TriangleArea(tri.va, tri.vb, tri.vc);
                maxArea = math.max(tri.area, maxArea);
                tri.indices = new int3(i0, i1, i2);

                // Outputs.
                delaEdges[delaEdgeCount++] = new int4(math.min(i0, i1), math.max(i0, i1), triangleCount, -1);
                delaEdges[delaEdgeCount++] = new int4(math.min(i1, i2), math.max(i1, i2), triangleCount, -1);
                delaEdges[delaEdgeCount++] = new int4(math.min(i2, i0), math.max(i2, i0), triangleCount, -1);
                triangles[triangleCount++] = tri;
            }
        }

        internal static void BuildTriangles(NativeArray<float2> vertices, int vertexCount, NativeArray<int> indices, int indexCount, ref NativeArray<UTriangle> triangles, ref int triangleCount, ref float maxArea)
        {
            // Check if there are invalid triangles or segments.
            for (int i = 0; i < indexCount; i += 3)
            {

                UTriangle tri = new UTriangle();
                var i0 = indices[i + 0];
                var i1 = indices[i + 1];
                var i2 = indices[i + 2];
                tri.va = vertices[i0];
                tri.vb = vertices[i1];
                tri.vc = vertices[i2];
                tri.c = UTess.CircumCircle(tri);
                tri.area = UTess.TriangleArea(tri.va, tri.vb, tri.vc);
                maxArea = math.max(tri.area, maxArea);
                triangles[triangleCount++] = tri;

            }
        }

        // From https://www.cs.cmu.edu/afs/cs/project/quake/public/code/predicates.c and is public domain. Can't find one within Unity.
        internal static float OrientFast(float2 a, float2 b, float2 c)
        {
            float epsilon = 1.1102230246251565e-16f;
            float errbound3 = (3.0f + 16.0f * epsilon) * epsilon;
            float l = (a.y - c.y) * (b.x - c.x);
            float r = (a.x - c.x) * (b.y - c.y);
            float det = l - r;
            float s = 0;
            if (l > 0)
            {
                if (r <= 0)
                {
                    return det;
                }
                else
                {
                    s = l + r;
                }
            }
            else if (l < 0)
            {
                if (r >= 0)
                {
                    return det;
                }
                else
                {
                    s = -(l + r);
                }
            }
            else
            {
                return det;
            }

            float tol = errbound3 * s;
            if (det >= tol || det <= -tol)
            {
                return det;
            }
            return epsilon;
        }

        // This is needed when doing PlanarGraph as it requires high precision separation of points.
        internal static double OrientFastDouble(double2 a, double2 b, double2 c)
        {
            double epsilon = 1.1102230246251565e-16f;
            double errbound3 = (3.0 + 16.0 * epsilon) * epsilon;
            double l = (a.y - c.y) * (b.x - c.x);
            double r = (a.x - c.x) * (b.y - c.y);
            double det = l - r;
            double s = 0;
            if (l > 0)
            {
                if (r <= 0)
                {
                    return det;
                }
                else
                {
                    s = l + r;
                }
            }
            else if (l < 0)
            {
                if (r >= 0)
                {
                    return det;
                }
                else
                {
                    s = -(l + r);
                }
            }
            else
            {
                return det;
            }

            double tol = errbound3 * s;
            if (det >= tol || det <= -tol)
            {
                return det;
            }
            return epsilon;
        }

        internal static UCircle CircumCircle(UTriangle tri)
        {
            float xa = tri.va.x * tri.va.x;
            float xb = tri.vb.x * tri.vb.x;
            float xc = tri.vc.x * tri.vc.x;
            float ya = tri.va.y * tri.va.y;
            float yb = tri.vb.y * tri.vb.y;
            float yc = tri.vc.y * tri.vc.y;
            float c = 2f * ((tri.vb.x - tri.va.x) * (tri.vc.y - tri.va.y) - (tri.vb.y - tri.va.y) * (tri.vc.x - tri.va.x));
            float x = ((tri.vc.y - tri.va.y) * (xb - xa + yb - ya) + (tri.va.y - tri.vb.y) * (xc - xa + yc - ya)) / c;
            float y = ((tri.va.x - tri.vc.x) * (xb - xa + yb - ya) + (tri.vb.x - tri.va.x) * (xc - xa + yc - ya)) / c;
            float vx = (tri.va.x - x);
            float vy = (tri.va.y - y);
            return new UCircle { center = new float2(x, y), radius = math.sqrt((vx * vx) + (vy * vy)) };
        }

        internal static bool IsInsideCircle(UCircle c, float2 v)
        {
            return math.distance(v, c.center) < c.radius;
        }

        internal static float TriangleArea(float2 va, float2 vb, float2 vc)
        {
            float3 a = new float3(va.x, va.y, 0);
            float3 b = new float3(vb.x, vb.y, 0);
            float3 c = new float3(vc.x, vc.y, 0);
            float3 v = math.cross(a - b, a - c);
            return math.abs(v.z) * 0.5f;
        }

        internal static bool IsInsideCircle(float2 a, float2 b, float2 c, float2 p)
        {
            float ab = math.dot(a, a);
            float cd = math.dot(b, b);
            float ef = math.dot(c, c);

            float ax = a.x;
            float ay = a.y;
            float bx = b.x;
            float by = b.y;
            float cx = c.x;
            float cy = c.y;

            float circum_x = (ab * (cy - by) + cd * (ay - cy) + ef * (by - ay)) /
                                (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));
            float circum_y = (ab * (cx - bx) + cd * (ax - cx) + ef * (bx - ax)) /
                                (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

            float2 circum = new float2();
            circum.x = circum_x / 2;
            circum.y = circum_y / 2;
            float circum_radius = math.distance(a, circum);
            float dist = math.distance(p, circum);
            return circum_radius - dist > 0.00001f;
        }

        internal static void Copy<T>(NativeArray<T> src, int srcIndex, NativeArray<T> dst, int dstIndex, int length)
            where T : struct
        {
            NativeArray<T>.Copy(src, srcIndex, dst, dstIndex, length);
        }

        internal static void Copy<T>(NativeArray<T> src, NativeArray<T> dst, int length)
            where T : struct
        {
            Copy(src, 0, dst, 0, length);
        }

        internal unsafe static void InsertionSort<T, U>(void* array, int lo, int hi, U comp)
            where T : struct where U : IComparer<T>
        {
            int i, j;
            T t;
            for (i = lo; i < hi; i++)
            {
                j = i;
                t = UnsafeUtility.ReadArrayElement<T>(array, i + 1);
                while (j >= lo && comp.Compare(t, UnsafeUtility.ReadArrayElement<T>(array, j)) < 0)
                {
                    UnsafeUtility.WriteArrayElement<T>(array, j + 1, UnsafeUtility.ReadArrayElement<T>(array, j));
                    j--;
                }
                UnsafeUtility.WriteArrayElement<T>(array, j + 1, t);
            }
        }

        static void CopyGraph(NativeArray<float2> srcPoints, int srcPointCount, ref NativeArray<float2> dstPoints, ref int dstPointCount, NativeArray<int2> srcEdges, int srcEdgeCount, ref NativeArray<int2> dstEdges, ref int dstEdgeCount)
        {
            dstEdgeCount = srcEdgeCount;
            dstPointCount = srcPointCount;
            UTess.Copy(srcEdges, dstEdges, srcEdgeCount);
            UTess.Copy(srcPoints, dstPoints, srcPointCount);
        }

        static void CopyGeometry(NativeArray<int> srcIndices, int srcIndexCount, ref NativeArray<int> dstIndices, ref int dstIndexCount, NativeArray<float2> srcVertices, int srcVertexCount, ref NativeArray<float2> dstVertices, ref int dstVertexCount)
        {
            dstIndexCount = srcIndexCount;
            dstVertexCount = srcVertexCount;
            UTess.Copy(srcIndices, dstIndices, srcIndexCount);
            UTess.Copy(srcVertices, dstVertices, srcVertexCount);
        }

        static void TransferOutput(NativeArray<int2> srcEdges, int srcEdgeCount, ref NativeArray<int2> dstEdges, ref int dstEdgeCount, NativeArray<int> srcIndices, int srcIndexCount, ref NativeArray<int> dstIndices, ref int dstIndexCount, NativeArray<float2> srcVertices, int srcVertexCount, ref NativeArray<float2> dstVertices, ref int dstVertexCount)
        {
            dstEdgeCount = srcEdgeCount;
            dstIndexCount = srcIndexCount;
            dstVertexCount = srcVertexCount;
            UTess.Copy(srcEdges, dstEdges, srcEdgeCount);
            UTess.Copy(srcIndices, dstIndices, srcIndexCount);
            UTess.Copy(srcVertices, dstVertices, srcVertexCount);
        }

        static void GraphConditioner(NativeArray<float2> points, ref NativeArray<float2> pgPoints, ref int pgPointCount, ref NativeArray<int2> pgEdges, ref int pgEdgeCount)
        {
            var min = new float2(math.INFINITY, math.INFINITY);
            var max = float2.zero;
            for (int i = 0; i < points.Length; ++i)
            {
                min = math.min(points[i], min);
                max = math.max(points[i], max);
            }
            var mid = (max - min) * 0.5f;
            var kNonRect = 0.0001f;
            // Construct a simple convex hull rect!.
            pgEdgeCount = 8;
            pgEdges[0] = new int2(0, 1); pgEdges[1] = new int2(1, 2); pgEdges[2] = new int2(2, 3); pgEdges[3] = new int2(3, 4);
            pgEdges[4] = new int2(4, 5); pgEdges[5] = new int2(5, 6); pgEdges[6] = new int2(6, 7); pgEdges[7] = new int2(7, 0);
            pgPointCount = 8;
            pgPoints[0] = new float2(min.x, min.y); pgPoints[1] = new float2(min.x - kNonRect, min.y + mid.y); pgPoints[2] = new float2(min.x, max.y); pgPoints[3] = new float2(min.x + mid.x, max.y + kNonRect);
            pgPoints[4] = new float2(max.x, max.y); pgPoints[5] = new float2(max.x + kNonRect, min.y + mid.y); pgPoints[6] = new float2(max.x, min.y); pgPoints[7] = new float2(min.x + mid.x, min.y - kNonRect);
        }
        
        public static float4 Tessellate(Allocator allocator, NativeArray<float2> points, NativeArray<int2> edges, ref NativeArray<float2> outVertices, ref int outVertexCount, ref NativeArray<int> outIndices, ref int outIndexCount, ref NativeArray<int2> outEdges, ref int outEdgeCount)
        {
            // Inputs are garbage, just early out.
            outEdgeCount = 0; outIndexCount = 0; outVertexCount = 0;
            if (points.Length == 0 || points.Length >= kMaxVertexCount)
                return float4.zero;

            // Ensure inputs form a proper PlanarGraph.
            bool validGraph = false;
            int pgEdgeCount = 0, pgPointCount = 0;
            float4 ret = float4.zero;
            NativeArray<int2> pgEdges = new NativeArray<int2>(kMaxEdgeCount, allocator);
            NativeArray<float2> pgPoints = new NativeArray<float2>(kMaxVertexCount, allocator);

            // Valid Edges and Paths, correct the Planar Graph. If invalid create a simple convex hull rect.
            validGraph = PlanarGraph.Validate(allocator, points, points.Length, edges, edges.Length, ref pgPoints, ref pgPointCount, ref pgEdges, ref pgEdgeCount);
            if (!validGraph)
                GraphConditioner(points, ref pgPoints, ref pgPointCount, ref pgEdges, ref pgEdgeCount);

            // Do a proper Delaunay Triangulation.
            int tsIndexCount = 0, tsVertexCount = 0;
            NativeArray<int> tsIndices = new NativeArray<int>(kMaxIndexCount, allocator);
            NativeArray<float2> tsVertices = new NativeArray<float2>(kMaxVertexCount, allocator);
            validGraph = Tessellator.Tessellate(allocator, pgPoints, pgPointCount, pgEdges, pgEdgeCount, ref tsVertices, ref tsVertexCount, ref tsIndices, ref tsIndexCount);
            if (!validGraph)
            {
                // Create a simplex convex hull rect.
                GraphConditioner(points, ref pgPoints, ref pgPointCount, ref pgEdges, ref pgEdgeCount);
                tsIndexCount = 0; tsVertexCount = 0;
                Tessellator.Tessellate(allocator, pgPoints, pgPointCount, pgEdges, pgEdgeCount, ref tsVertices, ref tsVertexCount, ref tsIndices, ref tsIndexCount);
            }                

            // Copy Out
            TransferOutput(pgEdges, pgEdgeCount, ref outEdges, ref outEdgeCount, tsIndices, tsIndexCount, ref outIndices, ref outIndexCount, tsVertices, tsVertexCount, ref outVertices, ref outVertexCount);            
            
            // Dispose Temp Memory. 
            tsVertices.Dispose();
            tsIndices.Dispose();
            pgPoints.Dispose();
            pgEdges.Dispose();
            return ret;
        }
        
        public static float4 Subdivide(Allocator allocator, NativeArray<float2> points, NativeArray<int2> edges, ref NativeArray<float2> outVertices, ref int outVertexCount, ref NativeArray<int> outIndices, ref int outIndexCount, ref NativeArray<int2> outEdges, ref int outEdgeCount, float areaFactor, float targetArea, int smoothenCount)
        {
            // Inputs are garbage, just early out.
            outEdgeCount = 0; outIndexCount = 0; outVertexCount = 0;
            if (points.Length == 0)
                return float4.zero;

            // Do a proper Delaunay Triangulation.
            float4 ret = float4.zero;
            int tsIndexCount = 0, tsVertexCount = 0;
            NativeArray<int> tsIndices = new NativeArray<int>(kMaxIndexCount, allocator);
            NativeArray<float2> tsVertices = new NativeArray<float2>(kMaxVertexCount, allocator);
            var validGraph = Tessellator.Tessellate(allocator, points, points.Length, edges, edges.Length, ref tsVertices, ref tsVertexCount, ref tsIndices, ref tsIndexCount);
            
            // Refinement and Smoothing. 
            bool refined = false;
            bool refinementRequired = (targetArea != 0 || areaFactor != 0);
            if (validGraph && refinementRequired)
            {
                // Do Refinement until success.
                float maxArea = 0;
                int rfEdgeCount = 0, rfPointCount = 0, rfIndexCount = 0, rfVertexCount = 0;
                NativeArray<int2> rfEdges = new NativeArray<int2>(kMaxEdgeCount, allocator);
                NativeArray<float2> rfPoints = new NativeArray<float2>(kMaxVertexCount, allocator);
                NativeArray<int> rfIndices = new NativeArray<int>(kMaxIndexCount, allocator);
                NativeArray<float2> rfVertices = new NativeArray<float2>(kMaxVertexCount, allocator);

                if (targetArea != 0)
                {
                    while (targetArea < kMaxArea)
                    {
                        // Do Mesh Refinement.
                        CopyGraph(points, points.Length, ref rfPoints, ref rfPointCount, edges, edges.Length, ref rfEdges, ref rfEdgeCount);
                        CopyGeometry(tsIndices, tsIndexCount, ref rfIndices, ref rfIndexCount, tsVertices, tsVertexCount, ref rfVertices, ref rfVertexCount);
                        refined = Refinery.Condition(allocator, areaFactor, targetArea, ref rfPoints, ref rfPointCount, ref rfEdges, ref rfEdgeCount, ref rfVertices, ref rfVertexCount, ref rfIndices, ref rfIndexCount, ref maxArea);

                        if (refined && rfIndexCount > rfPointCount)
                        {
                            // Copy Out
                            ret.x = areaFactor;
                            TransferOutput(rfEdges, rfEdgeCount, ref outEdges, ref outEdgeCount, rfIndices, rfIndexCount, ref outIndices, ref outIndexCount, rfVertices, rfVertexCount, ref outVertices, ref outVertexCount);
                            break;
                        }

                        refined = false;
                        targetArea = targetArea * 2.0f;
                    }
                }
                else if (areaFactor != 0)
                {
                    while (areaFactor < 0.8f)
                    {
                        // Do Mesh Refinement.
                        CopyGraph(points, points.Length, ref rfPoints, ref rfPointCount, edges, edges.Length, ref rfEdges, ref rfEdgeCount);
                        CopyGeometry(tsIndices, tsIndexCount, ref rfIndices, ref rfIndexCount, tsVertices, tsVertexCount, ref rfVertices, ref rfVertexCount);
                        refined = Refinery.Condition(allocator, areaFactor, targetArea, ref rfPoints, ref rfPointCount, ref rfEdges, ref rfEdgeCount, ref rfVertices, ref rfVertexCount, ref rfIndices, ref rfIndexCount, ref maxArea);

                        if (refined && rfIndexCount > rfPointCount)
                        {
                            // Copy Out
                            ret.x = areaFactor;
                            TransferOutput(rfEdges, rfEdgeCount, ref outEdges, ref outEdgeCount, rfIndices, rfIndexCount, ref outIndices, ref outIndexCount, rfVertices, rfVertexCount, ref outVertices, ref outVertexCount);
                            break;
                        }

                        refined = false;
                        areaFactor = areaFactor * 2.0f;
                    }
                }

                if (refined)
                {
                    // Smoothen. At this point only vertex relocation is allowed, not vertex addition/removal.
                    // Note: Only refined mesh contains Steiner points and we only smoothen these points.
                    smoothenCount = math.clamp(smoothenCount, 0, kMaxSmoothenIterations);
                    while (smoothenCount > 0)
                    {
                        var smoothen = Smoothen.Condition(allocator, ref rfPoints, rfPointCount, rfEdges, rfEdgeCount, ref rfVertices, ref rfVertexCount, ref rfIndices, ref rfIndexCount);
                        if (!smoothen)
                            break;
                        // Copy Out
                        ret.y = (float)(smoothenCount);
                        TransferOutput(rfEdges, rfEdgeCount, ref outEdges, ref outEdgeCount, rfIndices, rfIndexCount, ref outIndices, ref outIndexCount, rfVertices, rfVertexCount, ref outVertices, ref outVertexCount);
                        smoothenCount--;
                    }
                }

                rfVertices.Dispose();
                rfIndices.Dispose();
                rfPoints.Dispose();
                rfEdges.Dispose();
            }

            // Refinement failed but Graph succeeded.
            if (validGraph && !refined) 
            {
                // Copy Out
                TransferOutput(edges, edges.Length, ref outEdges, ref outEdgeCount, tsIndices, tsIndexCount, ref outIndices, ref outIndexCount, tsVertices, tsVertexCount, ref outVertices, ref outVertexCount);
            }

            // Dispose Temp Memory. 
            tsVertices.Dispose();
            tsIndices.Dispose();
            return ret;
        }        

    }

}
