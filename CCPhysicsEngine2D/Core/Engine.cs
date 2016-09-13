using System.Collections.Generic;
using System.Linq;
using CCPhysicsEngine2D.Base;
using CCPhysicsEngine2D.Common;

namespace CCPhysicsEngine2D.Core
{
    /// <summary>
    /// 引擎
    /// </summary>
    public class Engine
    {
        public int PositionIterations { get; set; } = 6;

        public int VelocityIterations { get; set; } = 4;

        public int ConstraintIterations { get; set; } = 2;

        public bool EnableSleep { get; set; } = false;

        public long Timestamp { get; set; }

        public double Timescale { get; set; } = 1;

        public World World { get; private set; } = new World();

        public void Update(double delta, double correction)
        {
            Timestamp += (long)(delta * Timescale);
            BodiesApplyGravity(World.AllBodies, World.Gravity);
            BodiesUpdate(World.AllBodies, delta, Timescale, correction, World.Bounds);
            if (World.Modified)
            {
                World.SetModified(false, false, true);
            }
            BodiesClearForces(World.AllBodies);
        }

        private void BodiesApplyGravity(IEnumerable<Body> allBodies, Gravity gravity)
        {
            foreach (var body in allBodies.Where(x => !(x.Static || x.Sleep)))
            {
                body.Force.X += body.Mass * gravity.Direction.X * gravity.Scaling;
                body.Force.Y += body.Mass * gravity.Direction.Y * gravity.Scaling;
            }
        }

        private void BodiesUpdate(IEnumerable<Body> allBodies, double delta, double timescale, double correction, Bound bounds)
        {
            foreach (var body in allBodies.Where(x => !(x.Static || x.Sleep)))
            {
                body.Update(delta, timescale, correction, bounds);
            }
        }

        private void BodiesClearForces(IEnumerable<Body> allBodies)
        {
            foreach (var body in allBodies)
            {
                body.Force.Clear();
                body.Torque = 0;
            }
        }

        public void Clear()
        {
            
        }
    }
}
