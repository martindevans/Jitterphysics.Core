using Jitter;
using Jitter.Collision;
using Jitter.LinearMath;
using UnityEngine;

namespace Assets.Scenes
{
    public class PhysicsWorld
        : MonoBehaviour
    {
        public World World { get; } = new World(new CollisionSystemSAP());
        public Vector3 Gravity = new Vector3(0, -9.8f, 0);

        public void OnEnable()
        {
            World.Gravity = new System.Numerics.Vector3(Gravity.x, Gravity.y, Gravity.z);
        }

        private void FixedUpdate()
        {
            World.Step(Time.fixedDeltaTime);
        }
    }
}
