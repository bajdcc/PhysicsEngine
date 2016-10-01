using System.Collections.Generic;

namespace CCPhysicsEngine2D.Common
{
    public class Contact
    {
        public string Id { get; set; }
        public Vertex Vertex { get; set; }
        public double NormalImpulse { get; set; }
        public double TangentImpulse { get; set; }
    }
}
