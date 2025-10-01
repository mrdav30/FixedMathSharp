using System;
using System.Linq;
using System.Runtime.CompilerServices;
using FixedMathSharp.Utility;
using Xunit;

namespace FixedMathSharp.Tests
{
    public class DeterministicRandomTests
    {
        // Helper: pull a sequence from NextU64 to compare streams
        private static ulong[] U64Seq(DeterministicRandom rng, int count)
        {
            var arr = new ulong[count];
            for (int i = 0; i < count; i++) arr[i] = rng.NextU64();
            return arr;
        }

        // Helper: pull a sequence from Next(int) to compare streams
        private static int[] IntSeq(DeterministicRandom rng, int count, int min, int max)
        {
            var arr = new int[count];
            for (int i = 0; i < count; i++) arr[i] = rng.Next(min, max);
            return arr;
        }

        [Fact]
        public void SameSeed_Yields_IdenticalSequences()
        {
            var a = new DeterministicRandom(123456789UL);
            var b = new DeterministicRandom(123456789UL);

            // Interleave various calls to ensure internal state advances identically
            Assert.Equal(a.NextU64(), b.NextU64());
            Assert.Equal(a.Next(), b.Next());
            Assert.Equal(a.Next(1000), b.Next(1000));
            Assert.Equal(a.Next(-50, 50), b.Next(-50, 50));
            Assert.Equal(a.NextDouble(), b.NextDouble(), 14);

            // Then compare a longer run of NextU64 to be extra sure
            var seqA = U64Seq(a, 32);
            var seqB = U64Seq(b, 32);
            Assert.Equal(seqA, seqB);
        }

        [Fact]
        public void DifferentSeeds_Yield_DifferentSequences()
        {
            var a = new DeterministicRandom(1UL);
            var b = new DeterministicRandom(2UL);

            // It's possible (but astronomically unlikely) that first value matches; check a window
            var seqA = U64Seq(a, 16);
            var seqB = U64Seq(b, 16);

            // Require at least one difference in the first 16 draws
            Assert.NotEqual(seqA, seqB);
        }

        [Fact]
        public void FromWorldFeature_IsStable_AndSeparatesByFeature_AndIndex()
        {
            ulong world = 0xDEADBEEFCAFEBABEUL;
            ulong featureOre = 0x4F5245UL;    // 'ORE'
            ulong featureRiver = 0x524956UL;  // 'RIV'

            // Stability: repeated construction yields same sequence
            var a1 = DeterministicRandom.FromWorldFeature(world, featureOre, index: 0);
            var a2 = DeterministicRandom.FromWorldFeature(world, featureOre, index: 0);
            Assert.Equal(U64Seq(a1, 8), U64Seq(a2, 8));

            // Different featureKey -> different stream
            var b = DeterministicRandom.FromWorldFeature(world, featureRiver, index: 0);
            Assert.NotEqual(U64Seq(a1, 8), U64Seq(b, 8));

            // Different index -> different stream
            var c = DeterministicRandom.FromWorldFeature(world, featureOre, index: 1);
            Assert.NotEqual(U64Seq(a1, 8), U64Seq(c, 8));
        }

        [Fact]
        public void Next_NoArg_IsNonNegative_AndWithinRange()
        {
            var rng = new DeterministicRandom(42UL);
            for (int i = 0; i < 1000; i++)
            {
                int v = rng.Next();
                Assert.InRange(v, 0, int.MaxValue);
            }
        }

        [Theory]
        [InlineData(1)]
        [InlineData(2)]
        [InlineData(7)]
        [InlineData(97)]
        [InlineData(int.MaxValue)]
        public void Next_MaxExclusive_RespectsBounds_AndThrows_OnInvalid(int maxExclusive)
        {
            var rng = new DeterministicRandom(9001UL);
            for (int i = 0; i < 4096; i++)
            {
                int v = rng.Next(maxExclusive);
                Assert.InRange(v, 0, maxExclusive - 1);
            }
        }

        [Fact]
        public void Next_MaxExclusive_Throws_WhenNonPositive()
        {
            var rng = new DeterministicRandom(1UL);
            Assert.Throws<ArgumentOutOfRangeException>(() => rng.Next(0));
            Assert.Throws<ArgumentOutOfRangeException>(() => rng.Next(-5));
        }

        [Fact]
        public void Next_MinMax_RespectsBounds_AndThrows_OnInvalidRange()
        {
            var rng = new DeterministicRandom(2024UL);

            for (int i = 0; i < 4096; i++)
            {
                int v = rng.Next(-10, 10);
                Assert.InRange(v, -10, 9);
            }

            Assert.Throws<ArgumentException>(() => rng.Next(5, 5));
            Assert.Throws<ArgumentException>(() => rng.Next(10, -10));
            Assert.Throws<ArgumentException>(() => rng.Next(10, 10));
        }

        [Fact]
        public void NextDouble_IsInUnitInterval()
        {
            var rng = new DeterministicRandom(123UL);
            for (int i = 0; i < 4096; i++)
            {
                double d = rng.NextDouble();
                Assert.True(d >= 0.0 && d < 1.0, "NextDouble() must be in [0,1)");
            }
        }

        [Fact]
        public void NextBytes_FillsEntireBuffer_AndAdvancesState()
        {
            var rng = new DeterministicRandom(55555UL);

            // Exercise both the fast 8-byte path and the tail path
            var sizes = new[] { 1, 7, 8, 9, 10, 15, 16, 17, 31, 32, 33 };
            byte[][] results = new byte[sizes.Length][];

            for (int i = 0; i < sizes.Length; i++)
            {
                var buf = new byte[sizes[i]];
                rng.NextBytes(buf);
                // Ensure something was written (not all zeros) and length correct
                Assert.Equal(sizes[i], buf.Length);
                Assert.True(buf.Any(b => b != 0) || sizes[i] == 0);
                results[i] = buf;
            }

            // Make sure successive calls produce different buffers (very high probability)
            for (int i = 1; i < sizes.Length; i++)
            {
                Assert.NotEqual(results[i - 1], results[i]);
            }
        }

        [Fact]
        public void Fixed64_ZeroToOne_IsWithinRange_AndReproducible()
        {
            var rng1 = new DeterministicRandom(777UL);
            var rng2 = new DeterministicRandom(777UL);

            for (int i = 0; i < 2048; i++)
            {
                var a = rng1.NextFixed6401(); // [0,1)
                var b = rng2.NextFixed6401();
                Assert.True(a >= Fixed64.Zero && a < Fixed64.One);
                Assert.Equal(a, b); // same seed, same draw order → identical
            }
        }

        [Fact]
        public void Fixed64_MaxExclusive_IsWithinRange_AndThrowsOnInvalid()
        {
            var rng = new DeterministicRandom(888UL);

            // Positive max
            var max = 5 * Fixed64.One;
            for (int i = 0; i < 2048; i++)
            {
                var v = rng.NextFixed64(max);
                Assert.True(v >= Fixed64.Zero && v < max);
            }

            // Invalid: max <= 0
            Assert.Throws<ArgumentOutOfRangeException>(() => rng.NextFixed64(Fixed64.Zero));
            Assert.Throws<ArgumentOutOfRangeException>(() => rng.NextFixed64(-Fixed64.One));
        }

        [Fact]
        public void Fixed64_MinMax_RespectsBounds_AndThrowsOnInvalidRange()
        {
            var rng = new DeterministicRandom(999UL);

            var min = -Fixed64.One;
            var max = 2 * Fixed64.One;

            for (int i = 0; i < 2048; i++)
            {
                var v = rng.NextFixed64(min, max);
                Assert.True(v >= min && v < max);
            }

            Assert.Throws<ArgumentException>(() => rng.NextFixed64(Fixed64.One, Fixed64.One));
            Assert.Throws<ArgumentException>(() => rng.NextFixed64(Fixed64.One, Fixed64.Zero));
        }

        [Fact]
        public void Interleaved_APIs_Stay_Deterministic()
        {
            // Ensure mixing different API calls doesn't desynchronize deterministic equality
            var a1 = new DeterministicRandom(1234UL);
            var a2 = new DeterministicRandom(1234UL);

            // Apply the same interleaving sequence on both instances
            int i1 = a1.Next(100);
            int i2 = a2.Next(100);
            Assert.Equal(i1, i2);

            double d1 = a1.NextDouble();
            double d2 = a2.NextDouble();
            Assert.Equal(d1, d2, 14);

            var buf1 = new byte[13];
            var buf2 = new byte[13];
            a1.NextBytes(buf1);
            a2.NextBytes(buf2);
            Assert.Equal(buf1, buf2);

            var f1 = a1.NextFixed64(-Fixed64.One, Fixed64.One);
            var f2 = a2.NextFixed64(-Fixed64.One, Fixed64.One);
            Assert.Equal(f1, f2);

            // And the streams continue to match:
            Assert.Equal(a1.NextU64(), a2.NextU64());
            Assert.Equal(a1.Next(), a2.Next());
        }

        [Fact]
        public void Next_Int_Bounds_Cover_CommonAndEdgeRanges()
        {
            var rng = new DeterministicRandom(3141592653UL);

            // Small bounds including 1 (degenerate but valid)
            foreach (int bound in new[] { 1, 2, 3, 4, 5, 7, 16, 31, 32, 33, 64, 127, 128, 129, 255, 256, 257 })
            {
                for (int i = 0; i < 512; i++)
                {
                    int v = rng.Next(bound);
                    Assert.InRange(v, 0, bound - 1);
                }
            }

            // Wide range near int.MaxValue to exercise rejection logic frequently
            for (int i = 0; i < 1024; i++)
            {
                int v = rng.Next(int.MaxValue);
                Assert.InRange(v, 0, int.MaxValue - 1);
            }
        }

        [Fact]
        public void Next_Fixed64_Covers_TypicalGameRanges()
        {
            var rng = new DeterministicRandom(0xFEEDFACEUL);

            // [0, 10)
            var ten = 10 * Fixed64.One;
            for (int i = 0; i < 2048; i++)
            {
                var v = rng.NextFixed64(ten);
                Assert.True(v >= Fixed64.Zero && v < ten);
            }

            // [-5, 5)
            var neg5 = -5 * Fixed64.One;
            var pos5 = 5 * Fixed64.One;
            for (int i = 0; i < 2048; i++)
            {
                var v = rng.NextFixed64(neg5, pos5);
                Assert.True(v >= neg5 && v < pos5);
            }
        }
    }
}