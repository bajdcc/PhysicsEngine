using System;
using System.Collections.Generic;
using System.Linq;
using CCPhysicsEngine2D.Base;

namespace CCPhysicsEngine2D.Common
{
    public class Vertices
    {
        public List<Vertex> Vertexes { get; private set; }

        public Vertices()
        {
            Vertexes = new List<Vertex>();
        }

        public Vertices(List<Vertex> vertexes)
        {
            Vertexes = vertexes;
        }

        public void Translate(Point pt)
        {
            foreach (var vertex in Vertexes)
            {
                vertex.X += pt.X;
                vertex.Y += pt.Y;
            }
        }

        public void Rotate(double angle, Point pos)
        {
            if (Math.Sign(angle) == 0)
                return;

            var cos = Math.Cos(angle);
            var sin = Math.Sin(angle);

            foreach (var vertex in Vertexes)
            {
                var dx = vertex.X - pos.X;
                var dy = vertex.Y - pos.Y;

                vertex.X = pos.X + (dx * cos - dy * sin);
                vertex.Y = pos.Y + (dx * sin + dy * cos);
            }
        }

        public static Vertices Create(Body body)
        {
            var vertices = new Vertices();
            foreach (var vertex in body.Vertices.Vertexes)
            {
                vertices.Vertexes.Add(new Vertex()
                {
                    X = vertex.X,
                    Y = vertex.Y,
                    Body = body,
                    Index = vertex.Index,
                    Internal = false
                });
            }
            return vertices;
        }

        public static Vertices Create(Body body, List<Point> paths)
        {
            var vertices = new Vertices();
            for (var i = 0; i < paths.Count; i++)
            {
                var path = paths[i];
                vertices.Vertexes.Add(new Vertex()
                {
                    X = path.X,
                    Y = path.Y,
                    Body = body,
                    Index = i,
                    Internal = false
                });
            }
            return vertices;
        }

        public Axes ToAxes()
        {
            var axes = new Axes();
            var pts = new Dictionary<string, Point>();

            for (var i = 0; i < Vertexes.Count; i++)
            {
                var j = (i == Vertexes.Count - 1) ? 0 : (i + 1);

                var normal = new Point(
                    Vertexes[j].Y - Vertexes[i].Y,
                    Vertexes[i].X - Vertexes[j].X);

                normal.Normalize();

                var gradient = Math.Sign(normal.Y) == 0 ? "INF" : (normal.X / normal.Y).ToString("F3");

                if (pts.ContainsKey(gradient))
                    pts.Add(gradient, normal);
            }

            foreach (var v in pts.Values)
            {
                axes.Points.Add(v);
            }

            return axes;
        }

        public double Area(bool signed = false)
        {
            var area = 0d;
            var j = Vertexes.Count - 1;

            for (var i = 0; i < Vertexes.Count; i++)
            {
                area += (Vertexes[j].X - Vertexes[i].X) * (Vertexes[j].Y + Vertexes[i].Y);
                j = i;
            }

            if (signed)
                return area / 2;

            return Math.Abs(area) / 2;
        }

        public Point Centre()
        {
            var area = Area(true);
            var centre = new Point();
            for (var i = 0; i < Vertexes.Count; i++)
            {
                var j = (i == Vertexes.Count - 1) ? 0 : (i + 1);
                var cross = Vertexes[i].Cross(Vertexes[j]);
                var temp = Vertexes[i].Add(Vertexes[j]) * cross;
                centre += temp;
            }
            return centre / (6 * area);
        }

        public double Inertia(double mass)
        {
            var numerator = 0d;
            var denominator = 0d;
            
            for (var i = 0; i < Vertexes.Count; i++)
            {
                var j = (i == Vertexes.Count - 1) ? 0 : (i + 1);
                var cross = Math.Abs(Vertexes[j].Cross(Vertexes[i]));
                numerator += cross * (Vertexes[j].Dot(Vertexes[j]) + Vertexes[j].Dot(Vertexes[i])) + Vertexes[i].Dot(Vertexes[i]);
                denominator += cross;
            }

            return (mass / 6) * (numerator / denominator);
        }

        public void ClockwiseSort()
        {
            var centre = Mean();
            Vertexes.Sort((a, b) => Math.Sign(centre.Angle(a) - centre.Angle(b)));
        }

        private Point Mean()
        {
            var centre = new Point();

            foreach (var vertex in Vertexes)
            {
                centre.X += vertex.X;
                centre.Y += vertex.Y;
            }

            centre /= Vertexes.Count;
            return centre;
        }

        public Vertices Hull()
        {
            Vertexes.Sort((a, b) =>
            {
                var dx = Math.Sign(a.X - b.X);
                return dx != 0 ? dx : Math.Sign(a.Y - b.Y);
            });
            
            var lower = new List<Vertex>();
            foreach (var vertex in Vertexes)
            {
                while (lower.Count >= 2
                       && lower[lower.Count - 2].Cross3(lower[lower.Count - 1], vertex) <= 0d)
                {
                    lower.RemoveAt(lower.Count - 1);
                }

                lower.Add(vertex);
            }
            
            var upper = new List<Vertex>();
            foreach (var vertex in Enumerable.Reverse(Vertexes))
            {
                while (upper.Count >= 2
                       && upper[upper.Count - 2].Cross3(upper[upper.Count - 1], vertex) <= 0d)
                {
                    upper.RemoveAt(upper.Count - 1);
                }

                upper.Add(vertex);
            }
            
            upper.RemoveAt(upper.Count - 1);
            lower.RemoveAt(lower.Count - 1);

            return new Vertices(upper.Concat(lower).ToList());
        }
    }

    public class Vertex
    {
        public double X { get; set; }
        public double Y { get; set; }
        public int Index { get; set; }
        public Body Body { get; set; }
        public bool Internal { get; set; }

        public double Cross(Vertex vertex)
        {
            return (X * vertex.Y) - (Y * vertex.X);
        }

        public Point Add(Vertex vertex)
        {
            return new Point(X + vertex.X, Y + vertex.Y);
        }

        public double Dot(Vertex vertex)
        {
            return (X * vertex.X) + (Y * vertex.Y);
        }

        public double Cross3(Vertex vertex1, Vertex vertex2)
        {
            return (vertex1.X - X) * (vertex2.Y - Y) - (vertex1.Y - Y) * (vertex2.X - X);
        }
    }
}
