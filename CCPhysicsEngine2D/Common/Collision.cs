using CCPhysicsEngine2D.Base;

namespace CCPhysicsEngine2D.Common
{
    public class Collision
    {
        public Body BodyA { get; set; }
        public Body BodyB { get; set; }
        public Body ParentA { get; set; }
        public Body ParentB { get; set; }
        public bool Collided { get; set; }
        public double Depth { get; set; }
        public Body AxisBody { get; set; }
        public int AxisNumber { get; set; }
        public bool Reused { get; set; }
        public Point Normal { get; set; }
        public Point Tangent { get; set; }
        public Point Penetration { get; set; }
        public Vertices Supports { get; set; }
    }
}
