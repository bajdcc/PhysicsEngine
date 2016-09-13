using System.Collections.Generic;
using System.Linq;
using CCPhysicsEngine2D.Common;

namespace CCPhysicsEngine2D.Base
{
    /// <summary>
    /// 组合
    /// </summary>
    public class Composite : ObjBase
    {
        public Composite Parent { get; private set; }

        public bool Modified { get; private set; }

        public List<Body> Bodies { get; private set; } = new List<Body>();

        public List<Composite> Composites { get; private set; } = new List<Composite>();

        public List<Constraint> Constraints { get; private set; } = new List<Constraint>();

        public void SetModified(bool isModified, bool updateParents = false, bool updateChildren = false)
        {
            Modified = isModified;
            if (updateParents)
            {
                Parent?.SetModified(isModified, true, updateChildren);
            }
            if (updateChildren)
            {
                foreach (var composite in Composites)
                {
                    composite.SetModified(isModified, updateParents, true);
                }
            }
        }

        public IEnumerable<Body> AllBodies =>
            Bodies.Union(Composites.SelectMany(composite => composite.AllBodies));

        public void Add(ObjBase obj)
        {
            switch (obj.Type)
            {
                case ObjType.Body:
                    AddBody(obj as Body);
                    break;
                case ObjType.Constraint:
                    break;
                case ObjType.Composite:
                    break;
            }
        }

        private void AddBody(Body body)
        {
            Bodies.Add(body);
            SetModified(true, true);
        }
    }
}
