using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using CCPhysicsEngine2D.Core;

namespace CCPhysicsEngine2D.Common
{
    public class Pairs
    {
        public Dictionary<string, Pair> Table { get; set; } = new Dictionary<string, Pair>();
        public List<Pair> PairList{ get; set; } = new List<Pair>();

        private List<Pair> _collisionStart = new List<Pair>();
        private List<Pair> _collisionEnd = new List<Pair>();
        private List<Pair> _collisionActive = new List<Pair>();

        public void Update(List<Collision> collisions, long timestamp)
        {
            var activePairIds = new HashSet<string>();

            _collisionStart.Clear();
            _collisionEnd.Clear();
            _collisionActive.Clear();

            foreach (var collision in collisions.Where(collision => collision.Collided))
            {
                var pairId = Helper.GetPairId(collision.BodyA, collision.BodyB);
                activePairIds.Add(pairId);

                Pair pair;
                if (Table.TryGetValue(pairId, out pair))
                {
                    if (pair.Active)
                    {
                        _collisionActive.Add(pair);
                    }
                    else
                    {
                        _collisionStart.Add(pair);
                    }

                    pair.Update(collision, timestamp);
                }
                else
                {
                    pair = new Pair();
                    pair.Update(collision, timestamp);
                    Table[pairId] = pair;

                    _collisionStart.Add(pair);
                    PairList.Add(pair);
                }
            }
            
            foreach (var pair in PairList.Where(pair => pair.Active && !activePairIds.Contains(pair.Id)))
            {
                pair.SetActive(false, timestamp);
                _collisionEnd.Add(pair);
            }
        }

        public void RemoveOld(long timestamp)
        {
            var newPairs = new List<Pair>();
            foreach (var pair in PairList)
            {
                var collision = pair.Collision;
                if (collision.BodyA.Sleep || collision.BodyB.Sleep)
                {
                    pair.TimeUpdate = timestamp;
                    continue;
                }
                if (timestamp - pair.TimeUpdate <= 1e7)
                {
                    newPairs.Add(pair);
                }
            }
            PairList = newPairs;
        }
    }
}
