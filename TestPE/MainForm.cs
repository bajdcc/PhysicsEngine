using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using CCPhysicsEngine2D.Base;
using CCPhysicsEngine2D.Common;
using CCPhysicsEngine2D.Core;

namespace TestPE
{
    public partial class MainForm : Form
    {
        private bool _taskRunning;
        private Engine engine;
        private Renderer render;
        private Runner runner;

        public MainForm()
        {
            InitializeComponent();
        }

        private void MainForm_Load(object sender, EventArgs e)
        {
            engine = new Engine();
            engine.World.Gravity.Direction.Y = 1;
            double x = 10, y = 10, offset = 5, left = 0, right = 1000, top = 0, bottom = 500, thick = 10;
            engine.World.Add(Factory.CreateRectangleBody(x + right/2, y + top - offset, right, thick, true));
            engine.World.Add(Factory.CreateRectangleBody(x + right/2, y + bottom + offset, right, thick, true));
            engine.World.Add(Factory.CreateRectangleBody(x + left - offset, y + bottom / 2, thick, bottom, true));
            engine.World.Add(Factory.CreateRectangleBody(x + right + offset, y + bottom / 2, thick, bottom, true));
            for (var i = 0; i < 4; i++)
            {
                for (var j = 0; j < 4; j++)
                {
                    engine.World.Add(Factory.CreateRectangleBody(50+i*80, 50+i*60, 70, 50));
                }
            }
            runner = new Runner(engine);
            render = new Renderer(engine, runner);
            pictureBox1.Image = render.Bitmap;
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            OnTimer();
        }

        private async void OnTimer()
        {
            if (_taskRunning) return;
            _taskRunning = true;
            _taskRunning = await Task.Run(() =>
            {
                if (runner.FrameCount%100 < 50)
                {
                    engine.World.Gravity.Direction.X = 0;
                    engine.World.Gravity.Direction.Y = 1;
                }
                else
                {
                    var r = new Random();
                    engine.World.Gravity.Direction.X = r.NextDouble()*Math.Sin(runner.FrameCount/10.0);
                    engine.World.Gravity.Direction.Y = r.NextDouble()*Math.Sin(runner.FrameCount/10.0);
                }
                runner.Update(DateTime.Now.Ticks);
                render.Render();
                try
                {
                    BeginInvoke(new Action(() => pictureBox1.Refresh()));
                }
                catch (Exception ex)
                {
                    Trace.TraceError(ex.Message);
                }
                return false;
            });
        }

        private void pictureBox1_MouseDown(object sender, MouseEventArgs e)
        {
        }

        private void pictureBox1_MouseUp(object sender, MouseEventArgs e)
        {
        }

        private void pictureBox1_MouseMove(object sender, MouseEventArgs e)
        {
        }

        private void pictureBox1_MouseEnter(object sender, EventArgs e)
        {
        }

        private void pictureBox1_MouseLeave(object sender, EventArgs e)
        {
        }
    }
}
