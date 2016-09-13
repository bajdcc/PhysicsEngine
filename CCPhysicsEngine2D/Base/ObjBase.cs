using CCPhysicsEngine2D.Common;

namespace CCPhysicsEngine2D.Base
{
    /// <summary>
    /// 对象基类
    /// </summary>
    public class ObjBase
    {
        /// <summary>
        /// 编号
        /// </summary>
        public Id Id { get; private set; } = Id.Create();
        
        /// <summary>
        /// 类型
        /// </summary>
        public ObjType Type { get; private set; }

        /// <summary>
        /// 标签
        /// </summary>
        public string Label { set; private get; }
    }
}
