namespace Jitter.Dynamics
{
    /// <summary>
    /// Determines how collisions are resolved between bodies
    /// </summary>
    public record Material
    {
        /// <summary>
        /// Default material to use for a RigidBody if no material is specified
        /// </summary>
        public static readonly Material Default = new Material();

        /// <summary>
        /// The coefficient of restitution is the ratio of the relative velocity of separation after collision to the relative velocity of approach before collision.
        /// </summary>
        public float Restitution { get; init; }

        /// <summary>
        /// Friction to use between static contacts
        /// </summary>
        public float StaticFriction { get; init; } = 0.6f;

        /// <summary>
        /// Friction to use between moving contacts
        /// </summary>
        public float KineticFriction { get; init; } = 0.3f;
    }
}
