using System;
using System.Collections.Generic;
using System.Linq;
using CCPhysicsEngine2D.Base;
using CCPhysicsEngine2D.Core;

namespace CCPhysicsEngine2D.Common
{
    public class Pair
    {
        public string Id { get; set; }
        public bool Active { get; set; } = true;
        public Collision Collision { get; set; }
        public long Timestamp { get; set; }
        public long TimeUpdate { get; set; }
        public double Friction { get; set; }
        public double Separation { get; set; }
        public double Slop { get; set; }
        public double Restitution { get; set; }
        public double FrictionStatic { get; set; }
        public Dictionary<string, Contact> Contacts { get; set; } = new Dictionary<string, Contact>();
        public List<Contact> ActiveContacts { get; set; } = new List<Contact>();

        private double _inverseMass;
        private double _separation;

        public void Update(Collision collision, long timestamp)
        {
            var parentB = collision.ParentB;
            var parentA = collision.ParentA;

            Id = Helper.GetPairId(collision.BodyA, collision.BodyB);
            TimeUpdate = timestamp;
            Collision = collision;
            _inverseMass = parentA.InverseMass + parentB.InverseMass;
            Friction = Math.Min(parentA.Friction, parentB.Friction);
            FrictionStatic = Math.Max(parentA.FrictionStatic, parentB.FrictionStatic);
            Restitution = Math.Max(parentA.Restitution, parentB.Restitution);
            Slop = Math.Max(parentA.Slop, parentB.Slop);
            ActiveContacts.Clear();

            if (collision.Collided)
            {
                foreach (var vertex in collision.Supports.Vertexes)
                {
                    var contactId = vertex.GetId();
                    Contact contact;

                    if (Contacts.TryGetValue(contactId, out contact))
                    {
                        ActiveContacts.Add(contact);
                    }
                    else
                    {
                        var newContact = new Contact()
                        {
                            Id = contactId,
                            Vertex = vertex
                        };
                        Contacts[contactId] = newContact;
                        ActiveContacts.Add(newContact);
                    }
                }

                _separation = collision.Depth;
                SetActive(true, timestamp);
            }
            else
            {
                if (Active)
                    SetActive(false, timestamp);
            }
        }

        public void SetActive(bool active, long timestamp)
        {
            if (active)
            {
                Active = true;
                Timestamp = timestamp;
            }
            else
            {
                Active = false;
            }
        }
    }
}
