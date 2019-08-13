using Unity.Entities;

[System.Serializable]
public struct SPHParticle : ISharedComponentData
{
    public float radius;
    public float smoothingRadius;
    public float smoothingRadiusSq;

    public float mass;

    public float restDensity;
    public float viscosity;
    public float gravityMult;

    public float drag;
}

public class SPHParticleComponent : SharedComponentDataProxy<SPHParticle> { }
