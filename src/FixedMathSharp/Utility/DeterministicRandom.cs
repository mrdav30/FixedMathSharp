using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Utility
{
    /// <summary>
    /// Fast, seedable, deterministic RNG suitable for lockstep sims and map gen.
    /// Uses xoroshiro128++ with splitmix64 seeding. No allocations, no time/GUID.
    /// </summary>
    public struct DeterministicRandom
    {
        // xoroshiro128++ state
        private ulong _s0;
        private ulong _s1;

        #region Construction / Seeding

        public DeterministicRandom(ulong seed)
        {
            // Expand a single seed into two 64-bit state words via splitmix64.
            _s0 = SplitMix64(ref seed);
            _s1 = SplitMix64(ref seed);

            // xoroshiro requires non-zero state; repair pathological seed.
            if (_s0 == 0UL && _s1 == 0UL)
                _s1 = 0x9E3779B97F4A7C15UL;
        }

        /// <summary>
        /// Create a stream deterministically 
        /// Derived from (worldSeed, featureKey[,index]).
        /// </summary>
        public static DeterministicRandom FromWorldFeature(ulong worldSeed, ulong featureKey, ulong index = 0)
        {
            // Simple reversible mix (swap for a stronger mix if required).
            ulong seed = Mix64(worldSeed, featureKey);
            seed = Mix64(seed, index);
            return new DeterministicRandom(seed);
        }

        #endregion

        #region Core PRNG

        /// <summary>
        /// xoroshiro128++ next 64 bits.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ulong NextU64()
        {
            ulong s0 = _s0, s1 = _s1;
            ulong result = RotL(s0 + s1, 17) + s0;

            s1 ^= s0;
            _s0 = RotL(s0, 49) ^ s1 ^ (s1 << 21); // a,b
            _s1 = RotL(s1, 28);                   // c

            return result;
        }

        /// <summary>
        /// Next non-negative Int32 in [0, int.MaxValue].
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Next()
        {
            // Take high bits for better quality; mask to 31 bits non-negative.
            return (int)(NextU64() >> 33);
        }

        /// <summary>
        /// Unbiased int in [0, maxExclusive).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Next(int maxExclusive)
        {
            return maxExclusive <= 0 
                ? throw new ArgumentOutOfRangeException(nameof(maxExclusive)) 
                : (int)NextBounded((uint)maxExclusive);
        }

        /// <summary>
        /// Unbiased int in [min, maxExclusive).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Next(int minInclusive, int maxExclusive)
        {
            if (minInclusive >= maxExclusive) 
                throw new ArgumentException("min >= max");
            uint range = (uint)(maxExclusive - minInclusive);
            return minInclusive + (int)NextBounded(range);
        }

        /// <summary>
        /// Double in [0,1).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public double NextDouble()
        {
            // 53 random bits -> [0,1)
            return (NextU64() >> 11) * (1.0 / (1UL << 53));
        }

        /// <summary>
        /// Fill span with random bytes.
        /// </summary>
        public void NextBytes(Span<byte> buffer)
        {
            int i = 0;
            while (i + 8 <= buffer.Length)
            {
                ulong v = NextU64();
                Unsafe.WriteUnaligned(ref buffer[i], v);
                i += 8;
            }
            if (i < buffer.Length)
            {
                ulong v = NextU64();
                while (i < buffer.Length)
                {
                    buffer[i++] = (byte)v;
                    v >>= 8;
                }
            }
        }

        #endregion

        #region Fixed64 helpers

        /// <summary>
        /// Random Fixed64 in [0,1).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 NextFixed6401()
        {
            // Produce a raw value in [0, One.m_rawValue)
            ulong rawOne = (ulong)Fixed64.One.m_rawValue;
            ulong r = NextBounded(rawOne);
            return Fixed64.FromRaw((long)r);
        }

        /// <summary>
        /// Random Fixed64 in [0, maxExclusive).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 NextFixed64(Fixed64 maxExclusive)
        {
            if (maxExclusive <= Fixed64.Zero)
                throw new ArgumentOutOfRangeException(nameof(maxExclusive), "max must be > 0");
            ulong rawMax = (ulong)maxExclusive.m_rawValue;
            ulong r = NextBounded(rawMax);
            return Fixed64.FromRaw((long)r);
        }

        /// <summary>
        /// Random Fixed64 in [minInclusive, maxExclusive).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 NextFixed64(Fixed64 minInclusive, Fixed64 maxExclusive)
        {
            if (minInclusive >= maxExclusive)
                throw new ArgumentException("min >= max");
            ulong span = (ulong)(maxExclusive.m_rawValue - minInclusive.m_rawValue);
            ulong r = NextBounded(span);
            return Fixed64.FromRaw((long)r + minInclusive.m_rawValue);
        }

        #endregion

        #region Internals: unbiased range, splitmix64, mixing, rotations

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private ulong NextBounded(ulong bound)
        {
            // Rejection to avoid modulo bias.
            // threshold = 2^64 % bound, but expressed as (-bound) % bound
            ulong threshold = unchecked((ulong)-(long)bound) % bound;
            while (true)
            {
                ulong r = NextU64();
                if (r >= threshold)
                    return r % bound;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ulong RotL(ulong x, int k) => (x << k) | (x >> (64 - k));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ulong SplitMix64(ref ulong state)
        {
            ulong z = (state += 0x9E3779B97F4A7C15UL);
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ulong Mix64(ulong a, ulong b)
        {
            // Simple reversible mix (variant of splitmix finalizer).
            ulong x = a ^ (b + 0x9E3779B97F4A7C15UL);
            x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9UL;
            x = (x ^ (x >> 27)) * 0x94D049BB133111EBUL;
            return x ^ (x >> 31);
        }

        #endregion
    }
}