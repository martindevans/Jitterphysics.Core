using System.Collections.Generic;
using System.Linq;
using Jitter.Collision;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using UnityEngine;

namespace Assets.Scenes.BasicGameObjects
{
    public class PhysicsCube
        : MonoBehaviour
    {
        public bool IsFixed;

        private IPhysicsWorld _world;
        private RigidBody _body;

        private static Dictionary<CollisionIsland, Color> _islands = new();

        private void Awake()
        {
            // Yes, this is evil. No, I don't care.
            _world = FindObjectsOfType<MonoBehaviour>().OfType<IPhysicsWorld>().First();
        }

        private void OnEnable()
        {
            var shape = new BoxShape(transform.localScale.x, transform.localScale.y, transform.localScale.z);
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
            var material = GetComponent<MeshRenderer>().material;
            if (_body.IsStaticOrInactive)
                material.color = Color.gray;
            else
                material.color = GetColor(_body.CollisionIsland);

            var pos = _body.Position;
            var q = _body.Orientation.ToQuaternion();

            transform.position = new Vector3(pos.X, pos.Y, pos.Z);
            transform.rotation = new Quaternion(q.X, q.Y, q.Z, q.W);
        }

        private void LateUpdate()
        {
            //_islands.Clear();
        }

        private static Color GetColor(CollisionIsland island)
        {
            if (island == null)
                return Color.white;

            if (!_islands.TryGetValue(island, out var color))
            {
                color = Color.HSVToRGB(Random.value, 0.9f, 0.8f);
                _islands[island] = color;
            }

            return color;
        }
    }
}
