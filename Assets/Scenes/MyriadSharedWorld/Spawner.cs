using System.Collections;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using System.Runtime.CompilerServices;
using Myriad.ECS.Command;
using Packages.me.martindevans.myriad_unity_integration.Runtime;
using UnityEngine;

namespace Assets.Scenes.MyriadSharedWorld
{
    public class Spawner
        : MonoBehaviour
    {
        public GameTimeWorldHost World;
        public GameObject Prefab;
        public float Delay = 0.125f;
        public int Count = 250;

        private CommandBuffer _buffer;

        private void OnEnable()
        {
            _buffer = new CommandBuffer(World.World);
            StartCoroutine(CoSpawn());
        }

        private IEnumerator CoSpawn()
        {
            for (var i = 0; i < Count; i++)
            {
                yield return new WaitForSeconds(Delay);

                var shape = new BoxShape(Prefab.transform.localScale.x, Prefab.transform.localScale.y, Prefab.transform.localScale.z);
                var body = new RigidBody(shape)
                {
                    IsStatic = false,
                    Position = new System.Numerics.Vector3(transform.position.x, transform.position.y, transform.position.z)
                             + new System.Numerics.Vector3(Random.value, Random.value, Random.value) * 2 - System.Numerics.Vector3.One,
                    Orientation = JMatrix.CreateFromQuaternion(new System.Numerics.Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)),
                    EnableSpeculativeContacts = true,
                };

                var go = Instantiate(Prefab);

                _buffer.Create()
                       .Set(new PhysicsBody { Body = body })
                       .Set(go.GetComponent<MyriadEntity>());
                _buffer.Playback().Dispose();
            }
        }
    }
}