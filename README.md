![](https://imgur.com/a/Zc4xOFA)
# Unity (ECS/Job System) SPH
Implementation of the SPH Algorithm (fluid simulation) in Unity, comparing singlethread and ECS/Job System performances.

Using Unity 2018.2.19f0

More info in this article: https://medium.com/@leomontes_60748/how-to-implement-a-fluid-simulation-on-the-cpu-with-unity-ecs-job-system-bf90a0f2724f


### How to:
You can select the scene from 'Assets/Scenes'. Once in the scene, you can hit play to see the simulation. You can change some settings in the script from the MANAGER GameObject.

### Settings:
**Single-thread:**
- Particle count
- Particle parameters*

**ECS, Job system:**
- Particle count
You can find the particle parameters* inside 'Assets/Job System/Prefabs' in the SPHSphereECS GameObject.


* Particle parameters:
  - radius
  - smoothing radius
  - rest density
  - gravity multiplier
  - mass
  - viscosity
  - drag
