using CCPhysicsEngine2D.Common;

namespace CCPhysicsEngine2D.Base
{
    /// <summary>
    /// 世界
    /// </summary>
    public class World : Composite
    {
        public Gravity Gravity { get; private set; } = new Gravity();

        public Bound Bounds { get; private set; } = new Bound();
    }

    public class Gravity
    {
        public Point Direction = new Point(0, 1);

        public double Scaling = 0.001;
    }
}
