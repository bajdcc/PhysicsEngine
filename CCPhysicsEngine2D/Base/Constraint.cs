using CCPhysicsEngine2D.Common;

namespace CCPhysicsEngine2D.Base
{
    /// <summary>
    /// 约束
    /// </summary>
    public class Constraint : ObjBase
    {
        public Body BodyA { get; set; }
        public Body BodyB { get; set; }
        public Point PointA { get; set; }
        public Point PointB { get; set; }
        public double AngleA { get; set; }
        public double AngleB { get; set; }
        public double Length { get; set; }
        public double Stiffness { get; set; } = 1d;
        public double AngularStiffness { get; set; } = 0d;

        public void Init()
        {
            if (BodyA != null && PointA == null)
                PointA = new Point();
            if (BodyB != null && PointB == null)
                PointB = new Point();

            var initialPointA = BodyA != null
                ? (BodyA.Position + PointA)
                : PointA;
            var initialPointB = BodyB != null
                ? (BodyB.Position + PointB)
                : PointB;
            Length = (initialPointA - initialPointB).Magnitude();
            AngleA = BodyA?.Angle ?? AngleA;
            AngleB = BodyB?.Angle ?? AngleB;
        }
    }
}
