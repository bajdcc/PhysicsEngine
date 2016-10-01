using System.Linq;

namespace CCPhysicsEngine2D.Common
{
    public class Bound
    {
        public Point Min { get; set; } = new Point(double.MinValue, double.MinValue);

        public Point Max { get; set; } = new Point(double.MaxValue, double.MaxValue);

        public void Update(Vertices vertices, Point velocity)
        {
            Min.X = vertices.Vertexes.Min(x => x.X);
            Max.X = vertices.Vertexes.Max(x => x.X);
            Min.Y = vertices.Vertexes.Min(x => x.Y);
            Max.Y = vertices.Vertexes.Max(x => x.Y);
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

        public override string ToString()
        {
            return Min.ToString() + Max.ToString();
        }
    }
}
