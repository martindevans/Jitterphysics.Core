using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using UnityEngine;

namespace Assets.Scenes.BasicGameObjects
{
    public class PhysicsSphere
        : MonoBehaviour
    {
        public bool IsFixed;

        private PhysicsWorld _world;
        private RigidBody _body;

        private void Awake()
        {
            _world = FindObjectOfType<PhysicsWorld>();
        }

        private void OnEnable()
        {
            var shape = new SphereShape(transform.localScale.x / 2);
            _body = new RigidBody(shape)
            {
                IsStatic = IsFixed,
                Position = new System.Numerics.Vector3(transform.position.x, transform.position.y, transform.position.z),
                Orientation = JMatrix.CreateFromQuaternion(new System.Numerics.Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)),
                EnableSpeculativeContacts = true,
            };

            _world.World.Add(_body);
        }

        private void OnDisable()
        {
            _world.World.Remove(_body);
            _body = null;
        }

        private void Update()
        {
            var pos = _body.Position;
            var q = _body.Orientation.ToQuaternion();

            transform.position = new Vector3(pos.X, pos.Y, pos.Z);
            transform.rotation = new Quaternion(q.X, q.Y, q.Z, q.W);
        }
    }
}
