using Jitter.Dynamics;
using UnityEngine;

namespace Assets.Scenes.BasicGameObjects
{
    public class PhysicsSweepCube
        : MonoBehaviour
    {
        private PhysicsWorld _world;

        private void Awake()
        {
            _world = FindObjectOfType<PhysicsWorld>();
        }

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying)
                return;

            Gizmos.DrawLine(transform.position, transform.position + transform.forward * 10);

            var hit = _world.World.CollisionSystem.Raycast(V(transform.position), V(transform.forward), RaycastHit, out var body, out var normal, out var frac);
            if (hit)
            {
                var pos = transform.position + transform.forward * frac;
                Gizmos.DrawSphere(pos, 1);
            }
        }

        private bool RaycastHit(RigidBody body, System.Numerics.Vector3 normal, float fraction)
        {
            return true;
        }

        private System.Numerics.Vector3 V(Vector3 v)
        {
            return new System.Numerics.Vector3(v.x, v.y, v.z);
        }

        private System.Numerics.Quaternion Q(Quaternion q)
        {
            return new System.Numerics.Quaternion(q.x, q.y, q.z, q.w);
        }
    }
}
