using System.Collections;
using UnityEngine;

namespace Assets.Scenes.BasicGameObjects
{
    public class Spawner
        : MonoBehaviour
    {
        public GameObject Prefab;

        public float Delay;
        public int Count = 1000;

        private void OnEnable()
        {
            StartCoroutine(CoSpawn());
        }

        private IEnumerator CoSpawn()
        {
            Prefab.SetActive(false);

            for (var i = 0; i < Count; i++)
            {
                yield return new WaitForSeconds(Delay);

                var go = Instantiate(Prefab);
                go.transform.position = transform.position + new Vector3(Random.value, Random.value, Random.value) * 2 - Vector3.one;
                go.SetActive(true);
            }
        }
    }
}
