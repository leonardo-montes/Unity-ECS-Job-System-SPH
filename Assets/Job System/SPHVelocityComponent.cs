using Unity.Entities;
using Unity.Mathematics;

[System.Serializable]
public struct SPHVelocity : IComponentData
{
    public float3 Value;
}

public class SPHVelocityComponent : ComponentDataProxy<SPHVelocity> { }
