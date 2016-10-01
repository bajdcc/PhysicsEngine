using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using CCPhysicsEngine2D.Common;
using Point = CCPhysicsEngine2D.Common.Point;

namespace CCPhysicsEngine2D.Base
{
    /// <summary>
    ///     物体
    /// </summary>
    public class Body : ObjBase
    {
        private List<Body> _parts = new List<Body>();
        private Point _position = new Point();
        private bool _static = false;
        private Point _velocity = new Point();
        private double _angle;
        private double _angularVelocity;
        private double _density = 0.001;
        private double _inertia;
        private double _mass = 0.1;
        private Vertices _vertices;
        private double _anglePrev;

        public Body Parent { get; set; }

        public void Init(List<Point> path)
        {
            Vertices = Vertices.Create(this, path);
            Parts.Add(this);
            Parent = this;
            Render = new BodyRenderStruct(this);
        }

        public Point Position
        {
            get { return _position; }
            set
            {
                var delta = value - _position;
                PositionPrev.Offset(delta);

                foreach (var part in Parts)
                {
                    part.Position.Offset(delta);
                    part.Vertices.Translate(delta);
                    part.Bounds.Update(part.Vertices, Velocity);
                }
            }
        }

        public Point PositionPrev { get; set; } = new Point();

        public Point PositionImpulse { get; set; } = new Point();

        public Point ConstraintImpulse { get; set; } = new Point();

        public double ConstraintAngle { get; set; }

        public Point Force { get; set; } = new Point();

        public Point Velocity
        {
            get { return _velocity; }
            set
            {
                PositionPrev.X = Position.X - value.X;
                PositionPrev.Y = Position.Y - value.Y;
                Velocity.X = value.X;
                Velocity.Y = value.Y;
                Speed = _velocity.Magnitude();
            }
        }

        public Bound Bounds { get; set; } = new Bound();

        public Bound Region { get; set; }

        public double AngularVelocity
        {
            get { return _angularVelocity; }
            set
            {
                AnglePrev = Angle - value;
                _angularVelocity = value;
                AngularSpeed = Math.Abs(_angularVelocity);
            }
        }

        public double Angle
        {
            get { return _angle; }
            set
            {
                _angle = value;
                var delta = value - _angle;
                AnglePrev += delta;

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

        public double AnglePrev
        {
            get { return _anglePrev; }
            set { _anglePrev = value; }
        }

        public double Area { get; set; }

        public double InverseMass { get; set; }

        public double Mass
        {
            get { return _mass; }
            set
            {
                _mass = value;
                InverseMass = 1/value;
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

        public double Slop { get; set; } = 0.05;

        public int TotalContacts { get; set; }

        public bool Visible { get; set; } = true;

        public bool Static
        {
            get { return _static; }
            set
            {
                if (value)
                {
                    _static = true;
                    foreach (var part in Parts)
                    {
                        part._static = true;
                        part.Restitution = 0;
                        part.Friction = 1;
                        part._mass = part._inertia = part._density = double.MaxValue;
                        part.InverseMass = part.InverseInertia = 0;
                        part.PositionPrev.X = part.Position.X;
                        part.PositionPrev.Y = part.Position.Y;
                        part.AnglePrev = part.Angle;
                        part._angularVelocity = 0;
                        part.Speed = 0;
                        part.AngularSpeed = 0;
                        part.Motion = 0;
                    }
                }
                else
                {
                    _static = false;
                    foreach (var part in Parts)
                    {
                        part._static = false;
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
                InverseInertia = 1/value;
            }
        }

        public double InverseInertia { get; set; }

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
                Parent = this;

                foreach (var part in parts.Where(x => x != this))
                {
                    part.Parent = this;
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
                Parent = this;
                Position.X = centre.X;
                Position.Y = centre.Y;
                PositionPrev.X = centre.X;
                PositionPrev.Y = centre.Y;

                Mass = mass;
                Inertia = inertia;
                Position = centre;
            }
        }

        public BodyRenderStruct Render { get; set; }

        public void Update(double delta, double timescale, double correction, Bound bounds)
        {
            if (Math.Sign(Mass) == 0) return;

            var deltaTimeSquared = Math.Pow(delta*timescale*Timescale, 2);

            var frictionAir = 1 - FrictionAir*timescale*Timescale;
            var velocityPrevX = Position.X - PositionPrev.X;
            var velocityPrevY = Position.Y - PositionPrev.Y;

            Velocity.X = velocityPrevX*frictionAir*correction + Force.X/Mass*deltaTimeSquared;
            Velocity.Y = velocityPrevY*frictionAir*correction + Force.Y/Mass*deltaTimeSquared;

            PositionPrev.X = Position.X;
            PositionPrev.Y = Position.Y;
            Position.X += Velocity.X;
            Position.Y += Velocity.Y;

            _angularVelocity = (Angle - AnglePrev)*frictionAir*correction + Torque/Inertia*deltaTimeSquared;
            AnglePrev = Angle;
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

    public class BodyRenderStruct
    {
        public Color Fill { get; set; }

        public Color Stroke { get; set; }

        public Point Sprite { get; set; }

        public BodyRenderStruct(Body body)
        {
            Fill = body.Static ? Color.FromArgb(0xee, 0xee, 0xee) : RandomColor.Random();
            Stroke = ShadeColor(Fill, -20);
            Sprite = (body.Bounds.Min - body.Position) / (body.Bounds.Max - body.Bounds.Min);
        }

        private Color ShadeColor(Color color, int percent)
        {
            var amount = (int) Math.Round(2.55*percent);
            var R = Math.Min(Math.Max(0, color.R + amount), 255);
            var G = Math.Min(Math.Max(0, color.G + amount), 255);
            var B = Math.Min(Math.Max(0, color.B + amount), 255);
            return Color.FromArgb(R, G, B);
        }
    }

    public static class RandomColor
    {
        private static Random _random = new Random();
        private static List<Color> _colors = new List<Color>();

        static RandomColor()
        {
            _colors.Add(Color.FromArgb(0x55, 0x62, 0x70));
            _colors.Add(Color.FromArgb(0x4e, 0xcd, 0xc4));
            _colors.Add(Color.FromArgb(0xc7, 0xf4, 0x64));
            _colors.Add(Color.FromArgb(0xff, 0x6b, 0x6b));
            _colors.Add(Color.FromArgb(0xc4, 0x4d, 0x58));
        }

        public static Color Random()
        {
            return _colors[_random.Next(_colors.Count - 1)];
        }
    }
}