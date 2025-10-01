using System;
using System.Threading;

namespace FixedMathSharp.Utility
{
    /// <summary>
    /// Deterministic per-thread RNG facade.
    /// </summary>
    [Obsolete("ThreadLocalRandom is deprecated. Use DeterministicRandom or DeterministicRandom.FromWorldFeature(...) for deterministic streams.", false)]
    public static class ThreadLocalRandom
    {
        private static ulong _rootSeed = 0;
        private static Func<int> _threadIndexProvider = null!;
        private static ThreadLocal<DeterministicRandom> _threadRng = null!;

        /// <summary>
        /// Initialize global deterministic seeding. 
        /// Provide a stable threadIndex (0..T-1) for each thread.
        /// </summary>
        public static void Initialize(ulong rootSeed, Func<int> threadIndexProvider)
        {
            _rootSeed = rootSeed;
            _threadIndexProvider = threadIndexProvider ?? throw new ArgumentNullException(nameof(threadIndexProvider));

            _threadRng = new ThreadLocal<DeterministicRandom>(() =>
            {
                int idx = _threadIndexProvider();
                // Derive a unique stream per thread deterministically from rootSeed + idx.
                return DeterministicRandom.FromWorldFeature(_rootSeed, (ulong)idx);
            });
        }

        /// <summary>
        /// Create a new independent RNG from a specific seed (does not affect thread Instance).
        /// </summary>
        public static DeterministicRandom NewRandom(ulong seed) => new(seed);

        /// <summary>
        /// Per-thread RNG instance (requires Initialize to be called first).
        /// </summary>
        public static DeterministicRandom Instance
        {
            get
            {
                if (_threadRng == null)
                    throw new InvalidOperationException("ThreadLocalRandom.Initialize(rootSeed, threadIndexProvider) must be called first.");
                return _threadRng.Value;
            }
        }

        #region Convenience mirrors

        public static int Next() => Instance.Next();
        public static int Next(int maxExclusive) => Instance.Next(maxExclusive);
        public static int Next(int minInclusive, int maxExclusive) => Instance.Next(minInclusive, maxExclusive);
        public static double NextDouble() => Instance.NextDouble();
        public static double NextDouble(double min, double max) => Instance.NextDouble() * (max - min) + min;

        public static void NextBytes(byte[] buffer)
        {
            if (buffer == null) throw new ArgumentNullException(nameof(buffer));
            Instance.NextBytes(buffer);
        }

        public static Fixed64 NextFixed6401() => Instance.NextFixed6401();
        public static Fixed64 NextFixed64(Fixed64 maxExclusive) => Instance.NextFixed64(maxExclusive);
        public static Fixed64 NextFixed64(Fixed64 minInclusive, Fixed64 maxExclusive) => Instance.NextFixed64(minInclusive, maxExclusive);

        #endregion
    }
}
