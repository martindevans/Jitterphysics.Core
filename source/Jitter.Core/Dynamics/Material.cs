namespace Jitter.Dynamics
{
    public class Material
    {
        public float Restitution { get; set; }

        public float StaticFriction { get; set; } = 0.6f;

        public float KineticFriction { get; set; } = 0.3f;
    }
}
