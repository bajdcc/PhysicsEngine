using System;
using System.Collections.Generic;
using System.Diagnostics;
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

        public bool Debug { get; set; } = true;

        public Dictionary<string, List<Body>> Buckets { get; set; } = new Dictionary<string, List<Body>>();

        private Dictionary<string, TmpPair> _tmpPairs = new Dictionary<string, TmpPair>();

        private List<TmpPair> _tmpPairsList = new List<TmpPair>();

        public static readonly Point BucketSize = new Point(40, 40);

        public void Update(double delta, double correction)
        {
            Timestamp += (long)(delta * Timescale);
            var bodies = World.AllBodies.ToList();
            var constraints = World.AllConstraints.ToList();

            BodiesApplyGravity(bodies, World.Gravity);
            BodiesUpdate(bodies, delta, Timescale, correction, World.Bounds);
            SolveConstraint(constraints, bodies);
            UpdateBroadphase(bodies, World.Modified);

            if (World.Modified)
            {
                World.SetModified(false, false, true);
            }

            var collisions = FindCollisions(_tmpPairsList);
            Pairs.Update(collisions, Timestamp);
            Pairs.RemoveOld(Timestamp);

            var pairs = Pairs.PairList.Where(pair => pair.Active).ToList();
            SolvePosition(pairs);
            SolveVelocity(pairs);
            BodiesClearForces(bodies);
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

        public Pairs Pairs { get; set; } = new Pairs();

        private List<Collision> FindCollisions(List<TmpPair> pairs)
        {
            var collisions = new List<Collision>();
            foreach (var _pair in pairs)
            {
                var bodyA = _pair.BodyA;
                var bodyB = _pair.BodyB;

                if ((bodyA.Static || bodyA.Sleep) && (bodyB.Static || bodyB.Sleep))
                    continue;

                if (!bodyA.Bounds.Intersect(bodyB.Bounds))
                    continue;
                foreach (var partA in bodyA.EnumParts())
                {
                    foreach (var partB in bodyB.EnumParts())
                    {
                        if ((partA != bodyA || partB != bodyB) && !partA.Bounds.Intersect(partB.Bounds))
                            continue;

                        var pairId = Helper.GetPairId(partA, partB);
                        Pair pair;
                        var previousCollision = Pairs.Table.TryGetValue(pairId, out pair) && pair.Active ? pair.Collision : null;
                        var collision = Collides(partA, partB, previousCollision);
                        if (collision.Collided)
                        {
                            collisions.Add(collision);
                        }
                    }
                }
            }
            return collisions;
        }

        class CollisionOverlap
        {
            public double Overlap { get; set; }
            public int AxisNumber { get; set; }
            public Point Axis { get; set; }

            public CollisionOverlap(double overlap, int axisNumber, Point axis)
            {
                Overlap = overlap;
                AxisNumber = axisNumber;
                Axis = axis;
            }
        }

        private Collision Collides(Body bodyA, Body bodyB, Collision collsionPrev)
        {
            Collision collision;
            CollisionOverlap minOverlap;
            var canReusePrevCol = false;

            if (collsionPrev != null)
            {
                var parentA = bodyA.Parent;
                var parentB = bodyB.Parent;
                var motion = parentA.Speed * parentA.Speed
                             + parentA.AngularSpeed * parentA.AngularSpeed
                             + parentB.Speed * parentB.Speed
                             + parentB.AngularSpeed * parentB.AngularSpeed;

                canReusePrevCol = collsionPrev.Collided && motion < 0.2;
                collision = collsionPrev;
            }
            else
            {
                collision = new Collision
                {
                    Collided = false,
                    BodyA = bodyA,
                    BodyB = bodyB
                };
            }

            if (collsionPrev != null && canReusePrevCol)
            {
                var axisBodyA = collision.AxisBody;
                var axisBodyB = axisBodyA == bodyA ? bodyB : bodyA;
                var axes = new Axes();
                axes.Points.Add(axisBodyA.Axes.Points[collsionPrev.AxisNumber]);

                minOverlap = OverlapAxes(axisBodyA.Vertices, axisBodyB.Vertices, axes);
                collision.Reused = true;

                if (minOverlap.Overlap <= 0)
                {
                    collision.Collided = false;
                    return collision;
                }
            }
            else
            {
                var overlapAB = OverlapAxes(bodyA.Vertices, bodyB.Vertices, bodyA.Axes);

                if (overlapAB.Overlap <= 0)
                {
                    collision.Collided = false;
                    return collision;
                }

                var overlapBA = OverlapAxes(bodyB.Vertices, bodyA.Vertices, bodyB.Axes);

                if (overlapBA.Overlap <= 0)
                {
                    collision.Collided = false;
                    return collision;
                }

                if (overlapAB.Overlap < overlapBA.Overlap)
                {
                    minOverlap = overlapAB;
                    collision.AxisBody = bodyA;
                }
                else
                {
                    minOverlap = overlapBA;
                    collision.AxisBody = bodyB;
                }
                
                collision.AxisNumber = minOverlap.AxisNumber;
            }

            collision.BodyA = bodyA.Id < bodyB.Id ? bodyA : bodyB;
            collision.BodyB = bodyA.Id < bodyB.Id ? bodyB : bodyA;
            collision.Collided = true;
            collision.Normal = minOverlap.Axis;
            collision.Depth = minOverlap.Overlap;
            collision.ParentA = collision.BodyA.Parent;
            collision.ParentB = collision.BodyB.Parent;

            bodyA = collision.BodyA;
            bodyB = collision.BodyB;
            
            if (collision.Normal.Dot(bodyB.Position - bodyA.Position) > 0)
                collision.Normal.Negate();

            collision.Tangent = collision.Normal.Perpendicular();
            collision.Penetration = collision.Normal*collision.Depth;

            var verticesB = FindSupports(bodyA, bodyB, collision.Normal);
            var supports = collision.Supports ?? new Vertices();
            supports.Vertexes.Clear();
            
            if (bodyA.Vertices.Contains(verticesB.Item1))
                supports.Vertexes.Add(verticesB.Item1);

            if (bodyA.Vertices.Contains(verticesB.Item2))
                supports.Vertexes.Add(verticesB.Item2);
            
            if (supports.Vertexes.Count < 2)
            {
                var verticesA = FindSupports(bodyB, bodyA, collision.Normal.Negative());

                if (bodyB.Vertices.Contains(verticesA.Item1))
                    supports.Vertexes.Add(verticesA.Item1);

                if (supports.Vertexes.Count < 2 && bodyB.Vertices.Contains(verticesA.Item2))
                    supports.Vertexes.Add(verticesA.Item2);
            }
            
            if (supports.Vertexes.Count == 0)
            {
                supports.Vertexes.Add(verticesB.Item1);
                supports.Vertexes.Add(verticesB.Item2);
            }

            collision.Supports = supports;
            return collision;
        }

        private static Tuple<Vertex, Vertex> FindSupports(Body bodyA, Body bodyB, Point normal)
        {
            var nearestDistance = double.MaxValue;
            var vertexToBody = new Point();
            var vertices = bodyB.Vertices;
            var bodyAPosition = bodyA.Position;
            var vertexA = vertices.Vertexes[0];
            var distance = 0d;
            
            foreach (var _vertex in vertices.Vertexes)
            {
                vertexToBody = _vertex.Sub(bodyAPosition);
                distance = -normal.Dot(vertexToBody);
                if (!(distance < nearestDistance)) continue;
                nearestDistance = distance;
                vertexA = _vertex;
            }

            var index = vertexA.Index;
            var prevIndex = index - 1 >= 0 ? index - 1 : vertices.Vertexes.Count - 1;
            var vertex = vertices.Vertexes[prevIndex];
            vertexToBody = vertex.Sub(bodyAPosition);
            nearestDistance = -normal.Dot(vertexToBody);
            var vertexB = vertex;
            var nextIndex = (index + 1) % vertices.Vertexes.Count;
            vertex = vertices.Vertexes[nextIndex];
            vertexToBody = vertex.Sub(bodyAPosition);
            distance = -normal.Dot(vertexToBody);
            if (distance < nearestDistance)
            {
                vertexB = vertex;
            }

            return new Tuple<Vertex, Vertex>(vertexA, vertexB);
        }

        private static CollisionOverlap OverlapAxes(Vertices verticesA, Vertices verticesB, Axes axes)
        {
            var projectionA = new Point();
            var projectionB = new Point();
            var overlap = double.MaxValue;
            var axisNumber = -1;

            for (var i = 0; i < axes.Points.Count; i++)
            {
                var axis = axes.Points[i];

                ProjectToAxis(projectionA, verticesA, axis);
                ProjectToAxis(projectionB, verticesB, axis);

                var ol = Math.Min(projectionA.Y - projectionB.X, projectionB.Y - projectionA.X);

                if (ol <= 0)
                {
                    return new CollisionOverlap(ol, 0, new Point());
                }

                if (ol < overlap)
                {
                    overlap = ol;
                    axisNumber = i;
                }
            }

            return new CollisionOverlap(overlap, axisNumber, axes.Points[axisNumber]);
        }

        private static void ProjectToAxis(Point projection, Vertices vertices, Point axis)
        {
            var dot = vertices.Vertexes.Select(vertex => vertex.Dot(axis)).ToList();
            projection.X = dot.Min();
            projection.Y = dot.Max();
        }

        class TmpPair
        {
            public Body BodyA { get; set; }
            public Body BodyB { get; set; }
            public int Count { get; set; }

            public TmpPair(Body bodyA, Body bodyB, int count)
            {
                BodyA = bodyA;
                BodyB = bodyB;
                Count = count;
            }
        }

        private Bound GetRegion(Body body)
        {
            return new Bound
            {
                Min = new Point(Math.Floor(body.Bounds.Min.X/BucketSize.X),
                    Math.Floor(body.Bounds.Min.Y/BucketSize.Y)),
                Max = new Point(Math.Floor(body.Bounds.Max.X/BucketSize.X),
                    Math.Floor(body.Bounds.Max.Y/BucketSize.Y))
            };
        }

        private void ClearBroadphase()
        {
            Buckets.Clear();
            _tmpPairs.Clear();
            _tmpPairsList.Clear();
        }

        private void UpdateBroadphase(List<Body> bodies, bool forceUpdate)
        {
            var gridChanged = false;
            foreach (var body in bodies
                .Where(body => forceUpdate || !body.Sleep)
                .Where(
                    body =>
                        !(body.Bounds.Max.X < World.Bounds.Min.X) && !(body.Bounds.Min.X > World.Bounds.Max.X) &&
                        !(body.Bounds.Max.Y < World.Bounds.Min.Y) && !(body.Bounds.Min.Y > World.Bounds.Max.Y)))
            {
                var newRegion = GetRegion(body);
                if (body.Region == null || newRegion.GetId() != body.Bounds.GetId() || forceUpdate)
                {
                    if (body.Region == null || forceUpdate)
                        body.Region = newRegion;
                    var union = newRegion.Union(body.Region);
                    for (var x = union.Min.X; x <= union.Max.X; x++)
                    {
                        for (var y = union.Min.Y; y <= union.Max.Y; y++)
                        {
                            var bucketId = x + "," + y;
                            var bucketExists = Buckets.ContainsKey(bucketId);
                            var isInsideNewRegion = newRegion.Contains(x, y);
                            var isInsideOldRegion = body.Region.Contains(x, y);
                            if (!isInsideNewRegion && isInsideOldRegion)
                            {
                                if (bucketExists)
                                {
                                    var bucket = Buckets[bucketId];
                                    bucket.Remove(body);
                                    foreach (var pair in from _body in bucket
                                        select Helper.GetPairId(body, _body)
                                        into pairId
                                        where _tmpPairs.ContainsKey(pairId)
                                        select _tmpPairs[pairId])
                                    {
                                        pair.Count--;
                                    }
                                }
                            }
                            if (body.Region == newRegion || (isInsideNewRegion && !isInsideOldRegion) || forceUpdate)
                            {
                                var bucket = bucketExists ? Buckets[bucketId] : new List<Body>();
                                if (!bucketExists)
                                    Buckets.Add(bucketId, bucket);
                                foreach (var bodyB in bucket)
                                {
                                    if (body.Id.Value == bodyB.Id.Value || (body.Static && bodyB.Static))
                                        continue;
                                    var pairId = Helper.GetPairId(body, bodyB);
                                    TmpPair pair;
                                    if (_tmpPairs.TryGetValue(pairId, out pair))
                                    {
                                        pair.Count++;
                                    }
                                    else
                                    {
                                        _tmpPairs.Add(pairId, new TmpPair(body, bodyB, 1));
                                    }
                                }
                                bucket.Add(body);
                            }
                        }
                    }
                    body.Region = newRegion;
                    gridChanged = true;
                }
            }
            if (gridChanged)
            {
                _tmpPairsList = _tmpPairs.Values.Where(x => x.Count > 0).ToList();
                _tmpPairs = _tmpPairs.Where(x => x.Value.Count > 0).ToDictionary(x => x.Key, x => x.Value);
            }
        }

        private void SolveConstraint(List<Constraint> constraints, List<Body> bodies)
        {
            for (var i = 0; i < ConstraintIterations; i++)
            {
                SolveConstraintInternal(constraints);
            }
            PostSolveConstraint(bodies);
        }

        private void SolveConstraintInternal(IEnumerable<Constraint> constraints)
        {
            foreach (var constraint in constraints)
            {
                var bodyA = constraint.BodyA;
                var bodyB = constraint.BodyB;
                var pointA = constraint.PointA;
                var pointB = constraint.PointB;
                
                if (bodyA != null && !bodyA.Static)
                {
                    constraint.PointA = pointA.Rotate(bodyA.Angle - constraint.AngleA);
                    constraint.AngleA = bodyA.Angle;
                }
                
                if (bodyB != null && !bodyB.Static)
                {
                    constraint.PointB = pointB.Rotate(bodyB.Angle - constraint.AngleB);
                    constraint.AngleB = bodyB.Angle;
                }

                var pointAWorld = pointA;
                var pointBWorld = pointB;

                if (bodyA != null) pointAWorld = bodyA.Position + pointA;
                if (bodyB != null) pointBWorld = bodyB.Position + pointB;

                if (pointAWorld == null || pointBWorld == null)
                    return;

                var delta = pointAWorld - pointBWorld;
                var currentLength = delta.Magnitude();
                
                if (Math.Sign(currentLength) == 0)
                    currentLength = 0.000001d;

                var difference = (currentLength - constraint.Length)/currentLength;
                var normal = delta/currentLength;
                var force = delta*(difference*0.5*constraint.Stiffness*Timescale*Timescale);
                
                if (Math.Abs(1 - (currentLength/constraint.Length)) < 0.001d*Timescale)
                    return;

                Point velocityPointA, velocityPointB, offsetA = null, offsetB = null;
                double oAn, oBn, bodyADenom, bodyBDenom;
                if (bodyA != null && !bodyA.Static)
                {
                    offsetA = pointAWorld - bodyA.Position + force;
                    
                    bodyA.Velocity = bodyA.Position - bodyA.PositionPrev;
                    bodyA.AngularVelocity = bodyA.Angle - bodyA.AnglePrev;

                    velocityPointA = bodyA.Velocity + (offsetA.Perpendicular()*bodyA.AngularVelocity);
                    oAn = offsetA.Dot(normal);
                    bodyADenom = bodyA.InverseMass + bodyA.InverseInertia*oAn*oAn;
                }
                else
                {
                    velocityPointA = new Point();
                    bodyADenom = bodyA?.InverseMass ?? 0;
                }

                if (bodyB != null && !bodyB.Static)
                {
                    offsetB = pointBWorld - bodyB.Position + force;

                    bodyB.Velocity = bodyB.Position - bodyB.PositionPrev;
                    bodyB.AngularVelocity = bodyB.Angle - bodyB.AnglePrev;

                    velocityPointB = bodyB.Velocity + (offsetB.Perpendicular() * bodyB.AngularVelocity);
                    oBn = offsetB.Dot(normal);
                    bodyBDenom = bodyB.InverseMass + bodyB.InverseInertia * oBn * oBn;
                }
                else
                {
                    velocityPointB =new Point();
                    bodyBDenom = bodyB?.InverseMass ?? 0;
                }

                var relativeVelocity = (velocityPointB - velocityPointA);
                var normalImpulse = normal.Dot(relativeVelocity)/(bodyADenom + bodyBDenom);

                if (normalImpulse > 0) normalImpulse = 0;

                var normalVelocity = normal*normalImpulse;

                double torque;

                if (bodyA != null && !bodyA.Static)
                {
                    torque = offsetA.Cross(normalVelocity)*bodyA.InverseInertia*
                             (1 - constraint.AngularStiffness);
                    
                    bodyA.ConstraintImpulse.X -= force.X;
                    bodyA.ConstraintImpulse.Y -= force.Y;
                    bodyA.ConstraintAngle += torque;
                    
                    bodyA.Position.X -= force.X;
                    bodyA.Position.Y -= force.Y;
                    bodyA.Angle += torque;
                }

                if (bodyB != null && !bodyB.Static)
                {
                    torque = offsetB.Cross(normalVelocity) * bodyB.InverseInertia *
                             (1 - constraint.AngularStiffness);
                    
                    bodyB.ConstraintImpulse.X += force.X;
                    bodyB.ConstraintImpulse.Y += force.Y;
                    bodyB.ConstraintAngle -= torque;
                    
                    bodyB.Position.X += force.X;
                    bodyB.Position.Y += force.Y;
                    bodyB.Angle -= torque;
                }
            }
        }

        private void PostSolveConstraint(List<Body> bodies)
        {
            foreach (var body in bodies.Where(body => !body.ConstraintImpulse.IsZero() || Math.Sign(body.ConstraintAngle) != 0))
            {
                var impulse = body.ConstraintImpulse;
                for (var i = 0; i < body.Parts.Count; i++)
                {
                    var part = body.Parts[i];
                    part.Vertices.Translate(impulse);

                    if (i > 0)
                    {
                        part.Position.X += impulse.X;
                        part.Position.Y += impulse.Y;
                    }

                    if (Math.Sign(body.ConstraintAngle) != 0)
                    {
                        part.Vertices.Rotate(body.ConstraintAngle, body.Position);
                        part.Axes.Rotate(body.ConstraintAngle);
                        if (i > 0)
                        {
                            part.Position.RotateAbout(body.ConstraintAngle, body.Position);
                        }
                    }

                    part.Bounds.Update(part.Vertices, body.Velocity);
                }

                body.ConstraintAngle = 0;
                impulse.X = 0;
                impulse.Y = 0;
            }
        }

        private void SolvePosition(List<Pair> pairs)
        {
            PreSolvePosition(pairs);
            for (var i = 0; i < PositionIterations; i++)
            {
                SolvePositionInternal(pairs);
            }
            PostSolvePosition();
        }

        private void PreSolvePosition(IEnumerable<Pair> pairs)
        {
            foreach (var pair in pairs)
            {
                var activeCount = pair.ActiveContacts.Count;
                pair.Collision.ParentA.TotalContacts += activeCount;
                pair.Collision.ParentB.TotalContacts += activeCount;
            }
        }

        private void SolvePositionInternal(List<Pair> pairs)
        {
            foreach (var pair in Pairs.PairList.Where(pair => pair.Active))
            {
                var collision = pair.Collision;
                var bodyA = collision.ParentA;
                var bodyB = collision.ParentB;
                var normal = collision.Normal;

                var bodyBtoA = bodyB.PositionImpulse - bodyA.PositionImpulse + collision.Penetration;

                pair.Separation = normal.Dot(bodyBtoA);
            }

            foreach (var pair in Pairs.PairList.Where(pair => pair.Active && pair.Separation >= 0))
            {
                var collision = pair.Collision;
                var bodyA = collision.ParentA;
                var bodyB = collision.ParentB;
                var normal = collision.Normal;
                var positionImpulse = (pair.Separation - pair.Slop)*Timescale;

                if (bodyA.Static || bodyB.Static)
                    positionImpulse *= 2;

                if (!(bodyA.Static || bodyA.Sleep))
                {
                    var contactShare = 0.9/bodyA.TotalContacts;
                    bodyA.PositionImpulse += normal*positionImpulse*contactShare;
                }

                if (!(bodyB.Static || bodyB.Sleep))
                {
                    var contactShare = 0.9/bodyB.TotalContacts;
                    bodyB.PositionImpulse -= normal*positionImpulse*contactShare;
                }
            }
        }

        private void PostSolvePosition()
        {
            foreach (var body in World.AllBodies)
            {
                body.TotalContacts = 0;

                if (Math.Sign(body.PositionImpulse.X) != 0 || Math.Sign(body.PositionImpulse.Y) != 0)
                {
                    foreach (var part in body.Parts)
                    {
                        part.Vertices.Translate(body.PositionImpulse);
                        part.Bounds.Update(part.Vertices, body.Velocity);
                        part.Position.X += body.PositionImpulse.X;
                        part.Position.Y += body.PositionImpulse.Y;
                    }
                    
                    body.PositionPrev.X += body.PositionImpulse.X;
                    body.PositionPrev.Y += body.PositionImpulse.Y;

                    if (body.PositionImpulse.Dot(body.Velocity) < 0d)
                    {
                        body.PositionImpulse.X = 0d;
                        body.PositionImpulse.Y = 0d;
                    }
                    else
                    {
                        body.PositionImpulse.X *= 0.8;
                        body.PositionImpulse.Y *= 0.8;
                    }
                }
            }
        }

        private void SolveVelocity(List<Pair> pairs)
        {
            PreSolveVelocity(pairs);
            for (var i = 0; i < PositionIterations; i++)
            {
                SolveVelocityInternal(pairs);
            }
        }

        private void PreSolveVelocity(List<Pair> pairs)
        {
            foreach (var pair in pairs)
            {
                var contacts = pair.ActiveContacts;
                var collision = pair.Collision;
                var bodyA = collision.ParentA;
                var bodyB = collision.ParentB;
                var normal = collision.Normal;
                var tangent = collision.Tangent;

                foreach (var contact in contacts)
                {
                    var contactVertex = contact.Vertex;
                    var normalImpulse = contact.NormalImpulse;
                    var tangentImpulse = contact.TangentImpulse;

                    if (Math.Sign(normalImpulse) != 0 || Math.Sign(tangentImpulse) != 0)
                    {
                        var impulse = normal*normalImpulse + tangent*tangentImpulse;

                        if (!(bodyA.Static || bodyA.Sleep))
                        {
                            var offset = contactVertex.Sub(bodyA.Position);
                            bodyA.PositionPrev.X += impulse.X*bodyA.InverseMass;
                            bodyA.PositionPrev.Y += impulse.Y*bodyA.InverseMass;
                            bodyA.AnglePrev += offset.Cross(impulse)*bodyA.InverseInertia;
                        }

                        if (!(bodyB.Static || bodyB.Sleep))
                        {
                            var offset = contactVertex.Sub(bodyB.Position);
                            bodyB.PositionPrev.X -= impulse.X*bodyB.InverseMass;
                            bodyB.PositionPrev.Y -= impulse.Y*bodyB.InverseMass;
                            bodyB.AnglePrev -= offset.Cross(impulse)*bodyB.InverseInertia;
                        }
                    }
                }
            }
        }

        private void SolveVelocityInternal(List<Pair> pairs)
        {
            foreach (var pair in pairs)
            {
                var timeScaleSquared = Timescale*Timescale;
                var collision = pair.Collision;
                var bodyA = collision.ParentA;
                var bodyB = collision.ParentB;
                var normal = collision.Normal;
                var tangent = collision.Tangent;
                var contacts = pair.ActiveContacts;
                var contactShare = 1d / contacts.Count;
                
                bodyA.Velocity = bodyA.Position - bodyA.PositionPrev;
                bodyB.Velocity = bodyB.Position - bodyB.PositionPrev;
                bodyA.AngularVelocity = bodyA.Angle - bodyA.AnglePrev;
                bodyB.AngularVelocity = bodyB.Angle - bodyB.AnglePrev;
                
                foreach (var contact in contacts)
                {
                    var contactVertex = contact.Vertex;
                    var offsetA = contactVertex.Sub(bodyA.Position);
                    var offsetB = contactVertex.Sub(bodyB.Position);
                    var velocityPointA = bodyA.Velocity + (offsetA.Perpendicular()*bodyA.AngularVelocity);
                    var velocityPointB = bodyB.Velocity + (offsetB.Perpendicular()*bodyB.AngularVelocity);
                    var relativeVelocity = velocityPointA - velocityPointB;
                    var normalVelocity = normal.Dot(relativeVelocity);

                    var tangentVelocity = tangent.Dot(relativeVelocity);
                    var tangentSpeed = Math.Abs(tangentVelocity);
                    var tangentVelocityDirection = Math.Sign(tangentVelocity);

                    var normalImpulse = (1 + pair.Restitution)*normalVelocity;
                    var normalForce = Helper.Clamp(pair.Separation + normalVelocity, 0, 1)*5;

                    var tangentImpulse = tangentVelocity;
                    var maxFriction = double.MaxValue;

                    if (tangentSpeed > pair.Friction*pair.FrictionStatic*normalForce*timeScaleSquared)
                    {
                        maxFriction = tangentSpeed;
                        tangentImpulse = Helper.Clamp(pair.Friction*tangentVelocityDirection*timeScaleSquared,
                            -maxFriction, maxFriction);
                    }

                    var oAcN = offsetA.Cross(normal);
                    var oBcN = offsetB.Cross(normal);
                    var share = contactShare/(bodyA.InverseMass + bodyB.InverseMass +
                                              bodyA.InverseInertia*oAcN*oAcN +
                                              bodyB.InverseInertia*oBcN*oBcN);

                    normalImpulse *= share;
                    tangentImpulse *= share;

                    if (normalVelocity < 0 && normalVelocity*normalVelocity > 4*timeScaleSquared)
                    {
                        contact.NormalImpulse = 0d;
                    }
                    else
                    {
                        var contactNormalImpulse = contact.NormalImpulse;
                        contact.NormalImpulse = Math.Min(contact.NormalImpulse + normalImpulse, 0);
                        normalImpulse = contact.NormalImpulse - contactNormalImpulse;
                    }

                    if (tangentVelocity*tangentVelocity > 6*timeScaleSquared)
                    {
                        contact.TangentImpulse = 0d;
                    }
                    else
                    {
                        var contactTangentImpulse = contact.TangentImpulse;
                        contact.TangentImpulse = Helper.Clamp(contact.TangentImpulse + tangentImpulse, -maxFriction,
                            maxFriction);
                        tangentImpulse = contact.TangentImpulse - contactTangentImpulse;
                    }

                    var impulse = normal*normalImpulse + tangent*tangentImpulse;
                    
                    if (!(bodyA.Static || bodyA.Sleep))
                    {
                        bodyA.PositionPrev += impulse*bodyA.InverseMass;
                        bodyA.AnglePrev += offsetA.Cross(impulse)*bodyA.InverseInertia;
                    }

                    if (!(bodyB.Static || bodyB.Sleep))
                    {
                        bodyB.PositionPrev -= impulse*bodyB.InverseMass;
                        bodyB.AnglePrev -= offsetB.Cross(impulse)*bodyB.InverseInertia;
                    }
                }
            }
        }
    }
}
