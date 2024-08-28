namespace Jitter.Dynamics
{
    /// <summary>
    /// Determines how collisions are resolved between bodies
    /// </summary>
    public class Material
    {
        /// <summary>
        /// The coefficient of restitution is the ratio of the relative velocity of separation after collision to the relative velocity of approach before collision.
        /// </summary>
        public float Restitution { get; set; }

        /// <summary>
        /// Friction to use between static contacts
        /// </summary>
        public float StaticFriction { get; set; } = 0.6f;

        /// <summary>
        /// Friction to use between moving contacts
        /// </summary>
        public float KineticFriction { get; set; } = 0.3f;
    }
}
