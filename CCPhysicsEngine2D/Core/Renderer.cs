using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using CCPhysicsEngine2D.Base;

namespace CCPhysicsEngine2D.Core
{
    public class Renderer
    {
        public Bitmap Bitmap { get; private set; }

        private static Size _renderSize = new Size(1200, 600);
        private Engine _engine;
        private Runner _runner;
        private Graphics _graphics;

        public Renderer(Engine engine, Runner runner)
        {
            _engine = engine;
            _runner = runner;
            Bitmap = new Bitmap(_renderSize.Width, _renderSize.Height);
            _graphics = Graphics.FromImage(Bitmap);
            _graphics.SmoothingMode = SmoothingMode.AntiAlias;
            _graphics.InterpolationMode = InterpolationMode.HighQualityBicubic;
            _graphics.CompositingQuality = CompositingQuality.HighQuality;
        }

        public void Render()
        {
            _graphics.Clear(Color.WhiteSmoke);

            RenderBodies();
            RenderCollisions();
            RenderInfo();
        }

        private void RenderBodies()
        {
            var bodies = _engine.World.AllBodies.Where(x => x.Visible).ToList();
            var parts = bodies.SelectMany(x => x.EnumParts().Where(y => y.Visible)).ToList();
            if (_engine.Debug)
            {
                foreach (var body in bodies)
                {
                    var bounds = body.Bounds.ToPointF();
                    _graphics.DrawPolygon(PenFactory.Get(Color.DarkGray), bounds);
                }
            }
            foreach (var part in parts)
            {
                var vertices = part.Vertices.ToPointF();
                _graphics.FillPolygon(BrushFactory.Get(part.Render.Fill), vertices);
                _graphics.DrawPolygon(PenFactory.Get(part.Render.Stroke), vertices);
            }
            /*foreach (var part in parts)
            {
                foreach (var axis in part.Axes.Points)
                {
                    _graphics.DrawLine(PenFactory.Get(Color.LightCyan),
                        (float) part.Position.X, (float) part.Position.Y,
                        (float) part.Position.X + 5f*(float) axis.X,
                        (float) part.Position.Y + 5f*(float) axis.Y);
                }
            }*/
            if (!_engine.Debug) return;
            foreach (var part in parts)
            {
                foreach (var axis in part.Axes.Points)
                {
                    var first = part.Vertices.Vertexes.First();
                    var last = part.Vertices.Vertexes.Last();
                    _graphics.DrawLine(PenFactory.Get(Color.Salmon),
                        (float)part.Position.X, (float)part.Position.Y,
                        ((float)first.X + (float)last.X) / 2f,
                        ((float)first.Y + (float)last.Y) / 2f);
                }
            }
            foreach (var part in parts)
            {
                _graphics.DrawArc(PenFactory.Get(Color.Indigo),
                    (float) part.Position.X - 1.5f, (float) part.Position.Y - 1.5f, 3f, 3f, 0f, 360f);
            }
            foreach (var body in bodies)
            {
                _graphics.DrawLine(PenFactory.Get(Color.CornflowerBlue),
                        (float)body.Position.X, (float)body.Position.Y,
                        (float)body.Position.X + 2f * (float)(body.Position.X - body.PositionPrev.X),
                        (float)body.Position.Y + 2f * (float)(body.Position.Y - body.PositionPrev.Y));
            }
        }

        private void RenderCollisions()
        {
            if (!_engine.Debug) return;
            var pairs = _engine.Pairs.PairList.Where(x => x.Active).ToList();
            foreach (var contact in pairs.SelectMany(x => x.ActiveContacts))
            {
                _graphics.DrawArc(PenFactory.Get(Color.BurlyWood), (float) contact.Vertex.X - 1.5f,
                    (float) contact.Vertex.Y - 1.5f, 3, 3, 0f, 360f);
            }
            foreach (var pair in pairs)
            {
                var contacts = pair.ActiveContacts;
                if (contacts.Count > 0)
                {
                    var normalPosX = contacts[0].Vertex.X;
                    var normalPosY = contacts[0].Vertex.Y;

                    if (contacts.Count == 2)
                    {
                        normalPosX = (normalPosX + contacts[1].Vertex.X)/2d;
                        normalPosY = (normalPosY + contacts[1].Vertex.Y)/2d;
                    }

                    var collision = pair.Collision;
                    if (collision.BodyB == collision.Supports.Vertexes[0].Body || collision.BodyA.Static)
                    {
                        _graphics.DrawLine(PenFactory.Get(Color.Chocolate),
                            (float) normalPosX - (float) collision.Normal.X*8f,
                            (float) normalPosY - (float) collision.Normal.Y*8f,
                            (float) normalPosX, (float) normalPosY);
                    }
                    else
                    {
                        _graphics.DrawLine(PenFactory.Get(Color.Chocolate),
                            (float) normalPosX + (float) collision.Normal.X*8f,
                            (float) normalPosY + (float) collision.Normal.Y*8f,
                            (float) normalPosX, (float) normalPosY);
                    }
                }
            }
            /*foreach (var bucket in _engine.Buckets.Where(x => x.Value.Count > 1))
            {
                var id = bucket.Key.Split(',');
                var x = Math.Round(1.0 * int.Parse(id[0]));
                var y = Math.Round(1.0 * int.Parse(id[1]));
                _graphics.DrawRectangle(PenFactory.Get(bucket.Value.Count > 1 ? Color.DarkOliveGreen : Color.Olive),
                    (float) x*(float) Engine.BucketSize.X,
                    (float) y*(float) Engine.BucketSize.Y,
                    (float) Engine.BucketSize.X,
                    (float) Engine.BucketSize.Y);
                _graphics.DrawString(bucket.Value.Count.ToString(), _font,
                    BrushFactory.Get(Color.DarkOliveGreen),
                    (float) x*(float) Engine.BucketSize.X,
                    (float) y*(float) Engine.BucketSize.Y);
            }*/
        }

        private Font _font = new Font(FontFamily.GenericSansSerif, 16);
        private void RenderInfo()
        {
            _graphics.DrawString($"FPS: {_runner.Fps:F} COLL: {_engine.Pairs.PairList.Count(x => x.Active)}", _font,
                BrushFactory.Get(Color.Black), new PointF(40f, 20f));
        }

        static class PenFactory
        {
            private static readonly Dictionary<Color, Pen> Dict = new Dictionary<Color, Pen>();

            public static Pen Get(Color color)
            {
                if (Dict.ContainsKey(color))
                    return Dict[color];
                var pen = new Pen(color);
                Dict.Add(color, pen);
                return pen;
            }
        }

        static class BrushFactory
        {
            private static readonly Dictionary<Color, Brush> Dict = new Dictionary<Color, Brush>();

            public static Brush Get(Color color)
            {
                if (Dict.ContainsKey(color))
                    return Dict[color];
                var brush = new SolidBrush(color);
                Dict.Add(color, brush);
                return brush;
            }
        }
    }
}