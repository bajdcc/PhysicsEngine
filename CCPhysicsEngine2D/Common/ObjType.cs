using System.ComponentModel;

namespace CCPhysicsEngine2D.Common
{
    /// <summary>
    /// 对象类型
    /// </summary>
    public enum ObjType
    {
        [Description("物体")]
        Body,
        [Description("约束")]
        Constraint,
        [Description("组合")]
        Composite,
    }

    internal static class KeywordTypeHelper
    {
        public static string GetDesc(this ObjType type)
        {
            return EnumHelper.GetEnumDescription<ObjType>(type);
        }
    }
}
