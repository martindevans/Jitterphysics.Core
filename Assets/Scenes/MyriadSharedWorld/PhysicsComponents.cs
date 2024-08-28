using Myriad.ECS;

namespace Assets.Scenes.MyriadSharedWorld
{
    public struct PhysicsBody
        : IComponent
    {
        public Jitter.Dynamics.RigidBody Body;
    }
}
