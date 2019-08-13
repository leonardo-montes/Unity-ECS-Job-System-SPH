using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;

public class SPHManager : MonoBehaviour
{

    // Import
    [Header("Import")]
    [SerializeField] private GameObject sphParticlePrefab = null;
    [SerializeField] private GameObject sphColliderPrefab = null;
    private EntityManager manager;

    // Properties
    [Header("Properties")]
    [SerializeField] private int amount = 5000;



    private void Start()
    {
        // Imoprt
        //manager = World.Active.GetOrCreateSystem<EntityManager>();
        manager = World.Active.EntityManager;

        // Setup
        AddColliders();
        AddParticles(amount);
    }



    private void AddParticles(int _amount)
    {
        NativeArray<Entity> entities = new NativeArray<Entity>(_amount, Allocator.Temp);
        manager.Instantiate(sphParticlePrefab, entities);

        for (int i = 0; i < _amount; i++)
        {
            manager.SetComponentData(entities[i], new Translation { Value = new float3(i % 16 + UnityEngine.Random.Range(-0.1f, 0.1f), 2 + (i / 16 / 16) * 1.1f, (i / 16) % 16) + UnityEngine.Random.Range(-0.1f, 0.1f) });
        }

        entities.Dispose();
    }



    private void AddColliders()
    {
        // Find all colliders
        GameObject[] colliders = GameObject.FindGameObjectsWithTag("SPHCollider");

        // Turn them into entities
        NativeArray<Entity> entities = new NativeArray<Entity>(colliders.Length, Allocator.Temp);
        manager.Instantiate(sphColliderPrefab, entities);

        // Set data
        for (int i = 0; i < colliders.Length; i++)
        {
            manager.SetComponentData(entities[i], new SPHCollider
            {
                position = colliders[i].transform.position,
                right = colliders[i].transform.right,
                up = colliders[i].transform.up,
                scale = new float2(colliders[i].transform.localScale.x / 2f, colliders[i].transform.localScale.y / 2f)
            });
        }

        // Done
        entities.Dispose();
    }
}
