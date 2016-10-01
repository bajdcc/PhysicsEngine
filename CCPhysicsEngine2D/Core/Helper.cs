using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using CCPhysicsEngine2D.Base;
using CCPhysicsEngine2D.Common;
using Point = CCPhysicsEngine2D.Common.Point;

namespace CCPhysicsEngine2D.Core
{
    static class Helper
    {
        public static string GetId(this Bound bound)
        {
            return bound.Min.ToString() + bound.Max.ToString();
        }

        public static Bound Union(this Bound bound, Bound bound2)
        {
            return new Bound()
            {
                Min = new Point(Math.Min(bound.Min.X, bound2.Min.X),
                    Math.Min(bound.Min.Y, bound2.Min.Y)),
                Max = new Point(Math.Max(bound.Max.X, bound2.Max.X),
                    Math.Max(bound.Max.Y, bound2.Max.Y))
            };
        }

        public static bool Contains(this Bound bound, double x, double y)
        {
            return (x >= bound.Min.X && x <= bound.Max.X
                    && y >= bound.Min.Y && y <= bound.Max.Y);
        }

        public static bool Intersect(this Bound bound, Bound bound2)
        {
            return (bound.Min.X <= bound2.Max.X && bound.Max.X >= bound2.Min.X
                    && bound.Max.Y >= bound2.Min.Y && bound.Min.Y <= bound2.Max.Y);
        }

        public static string GetPairId(Body bodyA, Body bodyB)
        {
            return bodyA.Id < bodyB.Id
                ? bodyA.Id + "_" + bodyB.Id
                : bodyB.Id + "_" + bodyA.Id;
        }

        public static IEnumerable<Body> EnumParts(this Body body)
        {
            for (var i = body.Parts.Count > 1 ? 1 : 0; i < body.Parts.Count; i++)
            {
                yield return body.Parts[i];
            }
        }

        public static double Dot(this Vertex vertex, Point point)
        {
            return (vertex.X * point.X) + (vertex.Y * point.Y);
        }

        public static double Dot(this Point lhs, Point rhs)
        {
            return lhs.X*rhs.X + lhs.Y*rhs.Y;
        }

        public static void Negate(this Point lhs)
        {
            lhs.X = -lhs.X;
            lhs.Y = -lhs.Y;
        }

        public static Point Negative(this Point lhs)
        {
            return new Point(-lhs.X, -lhs.Y);
        }

        public static Point Perpendicular(this Point lhs)
        {
            return new Point(-lhs.Y, lhs.X);
        }

        public static bool Contains(this Vertices vertices, Vertex vertex)
        {
            for (var i = 0; i < vertices.Vertexes.Count; i++)
            {
                var j = (i == vertices.Vertexes.Count - 1) ? 0 : (i + 1);
                var vertice = vertices.Vertexes[i];
                var nextVertice = vertices.Vertexes[j];
                if ((vertex.X - vertice.X) * (nextVertice.Y - vertice.Y) + (vertex.Y - vertice.Y) * (vertice.X - nextVertice.X) > 0)
                {
                    return false;
                }
            }

            return true;
        }

        public static string GetId(this Vertex vertex)
        {
            return vertex.Body.Id + "_" + vertex.Index;
        }

        public static double Cross(this Point point, Point point2)
        {
            return (point.X * point2.Y) - (point.Y * point2.X);
        }

        public static Point Add(this Vertex vertex, Point point)
        {
            return new Point(vertex.X + point.X, vertex.Y + point.Y);
        }

        public static Point Sub(this Vertex vertex, Point point)
        {
            return new Point(vertex.X - point.X, vertex.Y - point.Y);
        }

        public static double Clamp(double number, double min, double max)
        {
            return number < min ? min : (number > max ? max : number);
        }

        public static Point Rotate(this Point point, double angle)
        {
            if (Math.Sign(angle) == 0)
                return new Point(point.X, point.Y);

            var cos = Math.Cos(angle);
            var sin = Math.Sin(angle);
            return new Point(point.X*cos - point.Y*sin,
                point.X*sin + point.Y*cos);
        }

        public static bool IsZero(this Point point)
        {
            return Math.Sign(point.X) == 0 && Math.Sign(point.Y) == 0;
        }

        public static PointF[] ToPointF(this Bound bound)
        {
            return new List<double>()
            {
                bound.Min.X,
                bound.Max.X,
                bound.Max.X,
                bound.Min.X
            }.Zip(new List<double>()
            {
                bound.Min.Y,
                bound.Min.Y,
                bound.Max.Y,
                bound.Max.Y
            }, (a, b) => new PointF((float) a, (float) b)).ToArray();
        }
    }
}
