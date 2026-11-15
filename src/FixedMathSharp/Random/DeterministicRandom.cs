//=======================================================================
// DeterministicRandom.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Random
{
    /// <summary>
    /// Generates pseudorandom values from explicitly seeded xoroshiro128++ state.
    /// </summary>
    /// <remarks>
    /// Initialize the generator with a seed before drawing values. A default instance has
    /// all-zero state, which cannot advance and can cause bounded draws to loop indefinitely.
    /// Draws mutate the instance, and copying the struct copies the stream at its current position.
    /// The same seed and draw sequence produce the same values. See <see cref="NextBytes"/>
    /// for byte order when exchanging byte streams across architectures.
    /// </remarks>
    public struct DeterministicRandom
    {
        // xoroshiro128++ state
        private ulong _s0;
        private ulong _s1;

        #region Construction / Seeding

        /// <summary>
        /// Initializes a random stream from a seed.
        /// </summary>
        /// <remarks>
        /// The constructor expands the seed into two state words using SplitMix64.
        /// </remarks>
        /// <param name="seed">Any 64-bit unsigned seed, including zero.</param>
        public DeterministicRandom(ulong seed)
        {
            // Expand a single seed into two 64-bit state words via splitmix64.
            _s0 = SplitMix64(ref seed);
            _s1 = SplitMix64(ref seed);
        }

        /// <summary>Creates a stream from a world seed, feature key, and index.</summary>
        /// <param name="worldSeed">The seed shared by features in the world.</param>
        /// <param name="featureKey">The caller-supplied key for a feature.</param>
        /// <param name="index">The caller-supplied index within the feature. The default is zero.</param>
        /// <returns>A seeded stream determined by the three input values.</returns>
        public static DeterministicRandom FromWorldFeature(ulong worldSeed, ulong featureKey, ulong index = 0)
        {
            // Simple reversible mix (swap for a stronger mix if required).
            ulong seed = Mix64(worldSeed, featureKey);
            seed = Mix64(seed, index);
            return new DeterministicRandom(seed);
        }

        #endregion

        #region Core PRNG

        /// <summary>Advances the stream by one word.</summary>
        /// <returns>The next 64-bit unsigned value.</returns>
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

        /// <summary>Draws a nonnegative integer from the stream.</summary>
        /// <returns>An integer from zero through <see cref="int.MaxValue"/>, inclusive.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Next()
        {
            // Take high bits for better quality; mask to 31 bits non-negative.
            return (int)(NextU64() >> 33);
        }

        /// <summary>Draws a uniformly distributed integer below a positive bound.</summary>
        /// <param name="maxExclusive">The positive, exclusive upper bound.</param>
        /// <returns>An integer from zero up to the upper bound, excluding the bound.</returns>
        /// <exception cref="ArgumentOutOfRangeException"><paramref name="maxExclusive"/> is zero or negative. The stream does not advance.</exception>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Next(int maxExclusive)
        {
            return maxExclusive <= 0
                ? throw new ArgumentOutOfRangeException(nameof(maxExclusive))
                : (int)NextBounded((uint)maxExclusive);
        }

        /// <summary>Draws a uniformly distributed integer from the specified interval.</summary>
        /// <param name="minInclusive">The inclusive lower bound.</param>
        /// <param name="maxExclusive">The exclusive upper bound, greater than <paramref name="minInclusive"/>.</param>
        /// <returns>An integer within the half-open interval.</returns>
        /// <exception cref="ArgumentException">The lower bound is greater than or equal to the upper bound. The stream does not advance.</exception>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Next(int minInclusive, int maxExclusive)
        {
            if (minInclusive >= maxExclusive)
                throw new ArgumentException("min >= max");

            uint range = (uint)(maxExclusive - minInclusive);
            return minInclusive + (int)NextBounded(range);
        }

        /// <summary>Overwrites a span with random bytes.</summary>
        /// <param name="buffer">Caller-owned storage. Every byte in the span is replaced.</param>
        /// <remarks>
        /// Each full eight-byte block consumes one word and uses native byte order.
        /// A partial final block consumes one additional word, least significant byte first,
        /// and discards unused bytes. An empty span leaves the stream position unchanged.
        /// </remarks>
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

        /// <summary>Draws a uniformly distributed fixed-point fraction.</summary>
        /// <returns>A value from zero up to one, excluding one.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 NextFixed6401()
        {
            // Produce a raw value in [0, One.m_rawValue)
            ulong rawOne = (ulong)Fixed64.One.m_rawValue;
            ulong r = NextBounded(rawOne);
            return Fixed64.FromRaw((long)r);
        }

        /// <summary>Draws uniformly from the representable fixed-point values below a positive bound.</summary>
        /// <param name="maxExclusive">The positive, exclusive upper bound.</param>
        /// <returns>A value from zero up to the upper bound, excluding the bound.</returns>
        /// <exception cref="ArgumentOutOfRangeException"><paramref name="maxExclusive"/> is zero or negative. The stream does not advance.</exception>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64 NextFixed64(Fixed64 maxExclusive)
        {
            if (maxExclusive <= Fixed64.Zero)
                throw new ArgumentOutOfRangeException(nameof(maxExclusive), "max must be > 0");

            ulong rawMax = (ulong)maxExclusive.m_rawValue;
            ulong r = NextBounded(rawMax);
            return Fixed64.FromRaw((long)r);
        }

        /// <summary>Draws uniformly from the representable fixed-point values in the specified interval.</summary>
        /// <param name="minInclusive">The inclusive lower bound.</param>
        /// <param name="maxExclusive">The exclusive upper bound, greater than <paramref name="minInclusive"/>.</param>
        /// <returns>A value within the half-open interval.</returns>
        /// <exception cref="ArgumentException">The lower bound is greater than or equal to the upper bound. The stream does not advance.</exception>
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
