using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace CCPhysicsEngine2D.Core
{
    public class Renderer
    {
        public Bitmap Bitmap { get; private set; }

        private static Size _renderSize = new Size(800, 600);
        private Engine _engine;
        private Graphics _graphics;

        public Renderer(Engine engine)
        {
            _engine = engine;
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
        }

        private void RenderBodies()
        {
            foreach (var body in _engine.World.AllBodies)
            {
                if (!body.Visible)
                    continue;

                // handle compound Parts
                for (var k = body.Parts.Count > 1 ? 1 : 0; k < body.Parts.Count; k++)
                {
                    var part = body.Parts[k];

                    if (!part.Visible)
                        continue;

                    var vertices = part.Vertices.ToPointF();
                    _graphics.FillPolygon(BrushFactory.Get(part.Render.Fill), vertices);
                    _graphics.DrawPolygon(PenFactory.Get(part.Render.Stroke), vertices);
                }
            }
        }

        public static class PenFactory
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

        public static class BrushFactory
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