using System;
using System.Collections.Generic;
using System.Diagnostics;
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
        private Runner runner;

        public MainForm()
        {
            InitializeComponent();
        }

        private void MainForm_Load(object sender, EventArgs e)
        {
            engine = new Engine();
            var path = new List<Point> {new Point(0, 0), new Point(0, 40), new Point(40, 40), new Point(40, 0)};
            var body = new Body(path);
            engine.World.Add(body);
            runner = new Runner(engine);
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
                runner.Update(DateTime.Now.Ticks);
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
