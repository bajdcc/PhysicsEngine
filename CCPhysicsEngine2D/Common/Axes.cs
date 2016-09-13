using System;
using System.Collections.Generic;

namespace CCPhysicsEngine2D.Common
{
    public class Axes
    {
        public List<Point> Points { get; private set; } = new List<Point>();

        public void Rotate(double angle)
        {
            if (Math.Sign(angle) == 0)
                return;

            var cos = Math.Cos(angle);
            var sin = Math.Sin(angle);

            foreach (var axis in Points)
            {
                var xx = axis.X * cos - axis.Y * sin;
                axis.Y = axis.X * sin + axis.Y * cos;
                axis.X = xx;
            }
        }
    }
}
