using Assets.Scenes.BasicGameObjects;
using Jitter.Collision;
using Myriad.ECS;
using Myriad.ECS.Command;
using Myriad.ECS.Components;
using Myriad.ECS.Queries;
using Myriad.ECS.Systems;
using Myriad.ECS.Worlds;
using Packages.me.martindevans.myriad_unity_integration.Runtime;
using UnityEngine;

namespace Assets.Scenes.MyriadSharedWorld
{
    public class PhysicsSystems
        : WorldSystemGroup<GameTime>, IPhysicsWorld
    {
        public Jitter.World World { get; } = new Jitter.World(new CollisionSystemSAP())
        {
            Gravity = new System.Numerics.Vector3(0, -9.81f, 0),
        };

        protected override ISystemGroup<GameTime> CreateGroup(World world)
        {
            var cmd = new CommandBufferSystem<GameTime>(world);

            return new SystemGroup<GameTime>(
                "physics",
                new StepJitterPhysicsWorld(World),
                new BindToPhysicsWorld(world, World, cmd.Buffer),
                new CopyPhysicsTransformToUnityTransform(world),
                cmd
            );
        }
    }

    public class StepJitterPhysicsWorld
        : BaseSystem<GameTime>
    {
        private readonly Jitter.World _world;

        public StepJitterPhysicsWorld(Jitter.World world)
        {
            _world = world;
        }

        public override void Update(GameTime data)
        {
            _world.Step((float)data.DeltaTime, 1 / 60f, 8);
        }
    }

    public class BindToPhysicsWorld
        : BaseSystem<GameTime>
    {
        private readonly World _world;
        private readonly Jitter.World _physics;
        private readonly CommandBuffer _cmd;
        private readonly QueryDescription _query;

        public BindToPhysicsWorld(World world, Jitter.World physics, CommandBuffer cmd)
        {
            _world = world;
            _physics = physics;
            _cmd = cmd;
            _query = new QueryBuilder().Include<PhysicsBody>().Exclude<BoundToWorld>().Build(world);
        }

        public override void Update(GameTime data)
        {
            foreach (var (entity, body) in _world.Query<PhysicsBody>(_query))
            {
                _physics.Add(body.Ref.Body);

                _cmd.Set(entity, new BoundToWorld
                {
                    Self = entity,
                    PhysicsWorld = _physics
                });
            }
        }

        private struct BoundToWorld
            : IDisposableComponent
        {
            public Entity Self;
            public Jitter.World PhysicsWorld;

            public void Dispose(ref LazyCommandBuffer buffer)
            {
                var body = Self.GetComponentRef<PhysicsBody>(buffer.World);

                PhysicsWorld.Remove(body.Body);
                buffer.CommandBuffer.Remove<BoundToWorld>(Self);
            }
        }
    }

    public class CopyPhysicsTransformToUnityTransform
        : BaseSystem<GameTime>
    {
        private readonly World _world;

        public CopyPhysicsTransformToUnityTransform(World world)
        {
            _world = world;
        }

        public override void Update(GameTime data)
        {
            foreach (var (e, b, m) in _world.Query<PhysicsBody, MyriadEntity>())
            {
                var t = m.Ref.gameObject.transform;
                t.localPosition = new Vector3(b.Ref.Body.Position.X, b.Ref.Body.Position.Y, b.Ref.Body.Position.Z);

                var q = b.Ref.Body.Orientation.ToQuaternion();
                t.localRotation = new Quaternion(q.X, q.Y, q.Z, q.W);
            }
        }
    }
}
