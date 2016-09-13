namespace CCPhysicsEngine2D.Common
{
    public class Bound
    {
        public Point Min { get; set; } = new Point(double.MinValue, double.MinValue);

        public Point Max { get; set; } = new Point(double.MaxValue, double.MaxValue);

        public void Update(Vertices vertices, Point velocity)
        {
            Min.X = double.MaxValue;
            Max.X = double.MinValue;
            Min.Y = double.MaxValue;
            Max.Y = double.MinValue;
            foreach (var pt in vertices.Vertexes)
            {
                if (pt.X > Max.X) Max.X = pt.X;
                if (pt.X < Min.X) Min.X = pt.X;
                if (pt.Y > Max.Y) Max.Y = pt.Y;
                if (pt.Y < Min.Y) Min.Y = pt.Y;
            }
            if (velocity.X > 0)
            {
                Max.X += velocity.X;
            }
            else
            {
                Min.X += velocity.X;
            }
            if (velocity.Y > 0)
            {
                Max.Y += velocity.Y;
            }
            else
            {
                Min.Y += velocity.Y;
            }
        }
    }
}
