namespace CCPhysicsEngine2D.Core
{
    public class Runner
    {
        public double Fps { get; set; } = 60;

        public double Delta { get; set; } = 1000.0 / 60;

        public double DeltaMin { get; set; } = 1000.0 / 60;

        public double DeltaMax { get; set; } = 2000.0 / 60;

        public double TimeScalePrev { get; set; } = 1;

        public double Correction { get; set; } = 1;

        public bool Enable { get; set; } = true;

        public long Timestamp { get; private set; }

        public long FrameCount { get; set; }

        private double _timePrev;
        private long _frame;

        private Engine _engine;

        public Runner(Engine engine)
        {
            _engine = engine;
        }

        public void Update(long time)
        {
            var delta = time - _timePrev;
            _timePrev = time;
            delta = delta < DeltaMin ? DeltaMin : delta;
            delta = delta > DeltaMax ? DeltaMax : delta;
            var correction = delta / Delta;
            Delta = delta;
            if (TimeScalePrev > 0.0001)
                correction *= _engine.Timescale / TimeScalePrev;
            if (_engine.Timescale < 0.0001)
                correction = 0;
            TimeScalePrev = _engine.Timescale;
            Correction = correction;
            _frame++;
            FrameCount++;
            if (time - Timestamp > 1e6)
            {
                Fps = 1e7d * _frame /(time - Timestamp);
                Timestamp = time;
                _frame = 0;
            }
            _engine.Update(delta, correction);
        }
    }
}
