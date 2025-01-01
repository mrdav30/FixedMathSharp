using System;
using System.Threading;

namespace FixedMathSharp.Utility
{
    /// <summary>
    /// Provides thread-safe, deterministic random number generation for use in simulations, games, 
    /// and physics engines. This utility ensures randomness is consistent across multiple threads, 
    /// avoiding common pitfalls of shared Random instances and aiding reproducible calculations.
    /// </summary>
    public static class ThreadLocalRandom
    {
        /// <summary>
        /// Random number generator used to generate seeds,
        /// which are then used to create new random number
        /// generators on a per-thread basis.
        /// </summary>
        private static readonly Random globalRandom = new Random(Guid.NewGuid().GetHashCode());
        private static readonly object globalLock = new object();

        /// <summary>
        /// Random number generator
        /// </summary>
        private static readonly ThreadLocal<Random> threadRandom = new ThreadLocal<Random>(NewRandom);

        /// <summary>
        /// Creates a new instance of Random. The seed is derived
        /// from a global (static) instance of Random, rather
        /// than time.
        /// </summary>
        public static Random NewRandom()
        {
            lock (globalLock)
                return new Random(globalRandom.Next());
        }

        /// <summary>
        /// Returns an instance of Random which can be used freely
        /// within the current thread.
        /// </summary>
        public static Random Instance => threadRandom.Value;

        /// <summary>See <see cref="Random.Next()" /></summary>
        public static int Next()
        {
            return Instance.Next();
        }

        /// <summary>See <see cref="Random.Next(int)" /></summary>
        public static int Next(int maxValue)
        {
            return Instance.Next(maxValue);
        }

        /// <summary>See <see cref="Random.Next(int, int)" /></summary>
        public static int Next(int minValue, int maxValue)
        {
            return Instance.Next(minValue, maxValue);
        }

        /// <summary>
        ///  Returns a random Fixed64 number that is less than `max`.
        /// </summary>
        /// <param name="max"></param>
        public static Fixed64 NextFixed64(Fixed64 max = default)
        {
            if (max == Fixed64.Zero)
                throw new ArgumentException("Max value must be greater than zero.");

            byte[] buf = new byte[8];
            Instance.NextBytes(buf);

            // Use bitwise operation to ensure a non-negative long.
            long longRand = BitConverter.ToInt64(buf, 0) & long.MaxValue;

            return Fixed64.FromRaw(longRand % max.m_rawValue);
        }

        /// <summary>
        ///  Returns a random Fixed64 number that is greater than or equal to `min`, and less than `max`.
        /// </summary>
        /// <param name="min"></param>
        /// <param name="max"></param>
        public static Fixed64 NextFixed64(Fixed64 min, Fixed64 max)
        {
            if (min >= max)
                throw new ArgumentException("Min value must be less than max.");

            byte[] buf = new byte[8];
            Instance.NextBytes(buf);

            // Ensure non-negative random long.
            long longRand = BitConverter.ToInt64(buf, 0) & long.MaxValue;

            return Fixed64.FromRaw(longRand % (max.m_rawValue - min.m_rawValue)) + min;
        }

        /// <summary>See <see cref="Random.NextDouble()" /></summary>
        public static double NextDouble()
        {
            return Instance.NextDouble();
        }

        /// <summary>See <see cref="Random.NextDouble()" /></summary>
        public static double NextDouble(double min, double max)
        {
            return Instance.NextDouble() * (max - min) + min;
        }

        /// <summary>See <see cref="Random.NextBytes(byte[])" /></summary>
        public static void NextBytes(byte[] buffer)
        {
            Instance.NextBytes(buffer);
        }
    }
}