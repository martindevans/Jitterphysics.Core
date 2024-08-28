using Myriad.ECS.Systems;
using Myriad.ECS.Worlds;
using Packages.me.martindevans.myriad_unity_integration.Runtime;

namespace Assets.Scenes.MyriadSharedWorld
{
    public class UnitySystems
        : WorldSystemGroup<GameTime>
    {
        protected override ISystemGroup<GameTime> CreateGroup(World world)
        {
            return new SystemGroup<GameTime>(
                "unity",
                new MyriadEntityBindingSystem<GameTime>(world)
            );
        }
    }

    public class PhysicsSpawner
        : BaseSystem<GameTime>
    {
        public override void Update(GameTime data)
        {
            throw new System.NotImplementedException();
        }
    }
}
