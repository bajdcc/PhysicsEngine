namespace CCPhysicsEngine2D.Common
{
    /// <summary>
    /// ID
    /// </summary>
    public class Id
    {
        private static int _id;

        public int Value { get; private set; }

        private Id(int id)
        {
            Value = id;
        }

        public static Id Create()
        {
            return new Id(_id++);
        }

        public static bool operator <(Id a, Id b)
        {
            return a.Value < b.Value;
        }

        public static bool operator >(Id a, Id b)
        {
            return a.Value > b.Value;
        }

        public override string ToString()
        {
            return Value.ToString();
        }
    }
}
