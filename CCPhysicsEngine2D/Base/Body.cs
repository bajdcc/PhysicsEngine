using System;
using System.Collections.Generic;
using System.Linq;
using CCPhysicsEngine2D.Common;

namespace CCPhysicsEngine2D.Base
{
    /// <summary>
    ///     物体
    /// </summary>
    public class Body : ObjBase
    {
        private readonly List<Body> _parts = new List<Body>();
        private readonly Point _position = new Point();
        private readonly bool _static = false;
        private readonly Point _velocity = new Point();
        private readonly Point _positionPrev = new Point();
        private double _angle;
        private double _angularVelocity;
        private double _density = 0.001;
        private double _inertia;
        private double _mass = 0.1;
        private Vertices _vertices;
        private double _anglePrev;
        private double _inverseInertia;
        private double _inverseMass;
        private Body _parent;

        public Body(List<Point> path)
        {
            Vertices = Vertices.Create(this, path);
            Parts.Add(this);
        }

        public Point Position
        {
            get { return _position; }
            set
            {
                var delta = value - _position;
                _positionPrev.Offset(delta);

                foreach (var part in Parts)
                {
                    part.Position.Offset(delta);
                    part.Vertices.Translate(delta);
                    part.Bounds.Update(part.Vertices, Velocity);
                }
            }
        }

        public Point Force { get; set; } = new Point();

        public Point Velocity
        {
            get { return _velocity; }
            set
            {
                _positionPrev.X = Position.X - value.X;
                _positionPrev.Y = Position.Y - value.Y;
                Velocity.X = value.X;
                Velocity.Y = value.Y;
                Speed = _velocity.Magnitude();
            }
        }

        public Bound Bounds { get; set; } = new Bound();

        public double AngularVelocity
        {
            get { return _angularVelocity; }
            set
            {
                _anglePrev = Angle - value;
                _angularVelocity = value;
                AngularSpeed = Math.Abs(_angularVelocity);
            }
        }

        public double Angle
        {
            get { return _angle; }
            set
            {
                var delta = value - _angle;
                _anglePrev += delta;

                for (var i = 0; i < _parts.Count; i++)
                {
                    var part = _parts[i];
                    part._angle += delta;
                    part.Vertices.Rotate(delta, _position);
                    part.Axes.Rotate(delta);
                    part.Bounds.Update(part.Vertices, Velocity);
                    if (i > 0)
                    {
                        part.Position.RotateAbout(delta, Position);
                    }
                }
            }
        }

        public double Area { get; set; }

        public double Mass
        {
            get { return _mass; }
            set
            {
                _mass = value;
                _inverseMass = 1/value;
                _density = value/Area;
            }
        }

        public double Motion { get; set; }

        public double Torque { get; set; }

        public double Restitution { get; set; }

        public double Speed { get; set; }

        public double AngularSpeed { get; set; }

        public double Density
        {
            get { return _density; }
            set
            {
                _density = value;
                Mass = value*Area;
            }
        }

        public double Timescale { get; set; } = 1;

        public bool Visible { get; set; } = true;

        public bool Static
        {
            get { return _static; }
            set
            {
                if (value)
                {
                    foreach (var part in Parts)
                    {
                        part.Static = true;
                        part.Restitution = 0;
                        part.Friction = 1;
                        part._mass = part._inertia = part._density = double.MaxValue;
                        part._inverseMass = part._inverseInertia = 0;
                        part._positionPrev.X = part.Position.X;
                        part._positionPrev.Y = part.Position.Y;
                        part._anglePrev = part.Angle;
                        part._angularVelocity = 0;
                        part.Speed = 0;
                        part.AngularSpeed = 0;
                        part.Motion = 0;
                    }
                }
                else
                {
                    foreach (var part in Parts)
                    {
                        part.Static = false;
                    }
                }
            }
        }

        public bool Sleep { get; set; }

        public double Inertia
        {
            get { return _inertia; }
            set
            {
                _inertia = value;
                _inverseInertia = 1/value;
            }
        }

        public double Friction { get; set; } = 0.1;

        public double FrictionStatic { get; set; } = 0.5;

        public double FrictionAir { get; set; } = 0.01;

        public Vertices Vertices
        {
            get { return _vertices; }
            set
            {
                _vertices = value.Vertexes[0].Body == this ? value : Vertices.Create(this);
                Axes = Vertices.ToAxes();
                Area = Vertices.Area();
                Mass = Density*Area;
                Vertices.Translate(Vertices.Centre() * -1);
                Inertia = 4*Vertices.Inertia(Mass);
                Vertices.Translate(Position);
                Bounds.Update(Vertices, Velocity);
            }
        }

        public Axes Axes { get; set; }

        public List<Body> Parts
        {
            get { return _parts; }
            set
            {
                var parts = value.ToList();

                _parts.Add(this);
                _parent = this;

                foreach (var part in parts.Where(x => x != this))
                {
                    part._parent = this;
                    _parts.Add(part);
                }

                if (_parts.Count == 1)
                    return;

                var vertices = new Vertices(_parts.SelectMany(x => x.Vertices.Vertexes).ToList());
                vertices.ClockwiseSort();

                var hull = vertices.Hull();
                var hullCentre = hull.Centre();

                Vertices = hull;
                Vertices.Translate(hullCentre);

                var mass = 0d;
                var area = 0d;
                var inertia = 0d;
                var centre = new Point();

                for (var i = _parts.Count == 1 ? 0 : 1; i < _parts.Count; i++)
                {
                    var part = _parts[i];
                    mass += part.Mass;
                    area += part.Area;
                    inertia += part.Inertia;
                    centre += part.Position*(part.Mass >= double.MaxValue ? part.Mass : 1d);
                }

                centre /= mass >= double.MaxValue ? mass : _parts.Count;


                Area = area;
                _parent = this;
                Position.X = centre.X;
                Position.Y = centre.Y;
                _positionPrev.X = centre.X;
                _positionPrev.Y = centre.Y;

                Mass = mass;
                Inertia = inertia;
                Position = centre;
            }
        }

        public void Update(double delta, double timescale, double correction, Bound bounds)
        {
            if (Math.Sign(Mass) == 0) return;

            var deltaTimeSquared = Math.Pow(delta*timescale*Timescale, 2);

            var frictionAir = 1 - FrictionAir*timescale*Timescale;
            var velocityPrevX = Position.X - _positionPrev.X;
            var velocityPrevY = Position.Y - _positionPrev.Y;

            Velocity.X = velocityPrevX*frictionAir*correction + Force.X/Mass*deltaTimeSquared;
            Velocity.Y = velocityPrevY*frictionAir*correction + Force.Y/Mass*deltaTimeSquared;

            _positionPrev.X = Position.X;
            _positionPrev.Y = Position.Y;
            Position.X += Velocity.X;
            Position.Y += Velocity.Y;

            _angularVelocity = (Angle - _anglePrev)*frictionAir*correction + Torque/Inertia*deltaTimeSquared;
            _anglePrev = Angle;
            Angle += AngularVelocity;

            Speed = Velocity.Magnitude();
            AngularSpeed = Math.Abs(AngularVelocity);

            for (var i = 0; i < _parts.Count; i++)
            {
                var part = _parts[i];
                part.Vertices.Translate(Velocity);
                if (i > 0)
                {
                    part._position.Offset(Velocity);
                }
                part.Vertices.Rotate(AngularVelocity, Position);
                part.Axes.Rotate(AngularVelocity);
                if (i > 0)
                {
                    part._position.RotateAbout(AngularVelocity, _position);
                }
                part.Bounds.Update(Vertices, Velocity);
            }
        }
    }
}