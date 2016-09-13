namespace CCPhysicsEngine2D.Common
{
    /// <summary>
    /// _id
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
    }
}
