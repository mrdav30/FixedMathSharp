using BenchmarkDotNet.Attributes;

namespace FixedMathSharp.Benchmarks;

[MemoryDiagnoser]
public class Fixed64ArithmeticBenchmarks
{
    private readonly Fixed64[] _left = BenchmarkFixtures.ScalarsA;
    private readonly Fixed64[] _right = BenchmarkFixtures.ScalarsB;
    private readonly Fixed64[] _integerScalars = BenchmarkFixtures.IntegerScalars;
    private readonly Fixed64[] _positive = BenchmarkFixtures.PositiveScalars;
    private readonly Fixed64[] _unit = BenchmarkFixtures.UnitScalars;
    private readonly Fixed64[] _angles = BenchmarkFixtures.Angles;
    private readonly Fixed64[] _degreeAngles = BenchmarkFixtures.DegreeAngles;
    private readonly Fixed64[] _tangentAngles = BenchmarkFixtures.TangentAngles;
    private static readonly Fixed64 s_rescuedMultiplyDivideValue = new(65_536);

    [Benchmark]
    public Fixed64 AddSubtractMultiply()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += (_left[i] + _right[i]) * Fixed64.Half - _right[i];

        return accumulator;
    }

    [Benchmark]
    public Fixed64 TryAddSubtractExact()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            if (Fixed64.TryAdd(_left[i], _right[i], out Fixed64 sum)
                && Fixed64.TrySubtract(_left[i], _right[i], out Fixed64 difference))
            {
                accumulator += sum + difference;
            }
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 AddSubtractInverseChecked()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Fixed64 sum = _left[i] + _right[i];
            Fixed64 difference = _left[i] - _right[i];
            if (sum - _right[i] == _left[i] && difference + _right[i] == _left[i])
                accumulator += sum + difference;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Divide()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += _left[i] / _positive[i];

        return accumulator;
    }

    [Benchmark]
    public Fixed64 MultiplyDivideOperatorChainOrdinary()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += (_left[i] * _right[i]) / _positive[i];

        return accumulator;
    }

    [Benchmark]
    public Fixed64 MultiplyDivideOperatorChainRescuedSaturation()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < BenchmarkFixtures.SampleCount; i++)
        {
            accumulator += (s_rescuedMultiplyDivideValue * s_rescuedMultiplyDivideValue)
                / s_rescuedMultiplyDivideValue;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 TryMultiplyDivideTwoFactorOrdinary()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            if (Fixed64.TryMultiplyDivide(
                _left[i],
                _right[i],
                _positive[i],
                out Fixed64 result))
            {
                accumulator += result;
            }
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 TryMultiplyDivideTwoFactorRescuedSaturation()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < BenchmarkFixtures.SampleCount; i++)
        {
            if (Fixed64.TryMultiplyDivide(
                s_rescuedMultiplyDivideValue,
                s_rescuedMultiplyDivideValue,
                s_rescuedMultiplyDivideValue,
                out Fixed64 result))
            {
                accumulator += result;
            }
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 MultiplyDivideOperatorChainThreeFactorOrdinary()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Fixed64 divisor = _positive[(i + 37) & (BenchmarkFixtures.SampleCount - 1)];
            accumulator += ((_left[i] * _right[i]) * _positive[i]) / divisor;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 TryMultiplyDivideThreeFactorOrdinary()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
        {
            Fixed64 divisor = _positive[(i + 37) & (BenchmarkFixtures.SampleCount - 1)];
            if (Fixed64.TryMultiplyDivide(
                _left[i],
                _right[i],
                _positive[i],
                divisor,
                out Fixed64 result))
            {
                accumulator += result;
            }
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 MultiplyDivideOperatorChainThreeFactorRescuedSaturation()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < BenchmarkFixtures.SampleCount; i++)
        {
            accumulator += ((s_rescuedMultiplyDivideValue * s_rescuedMultiplyDivideValue)
                * Fixed64.One) / s_rescuedMultiplyDivideValue;
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 TryMultiplyDivideThreeFactorRescuedSaturation()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < BenchmarkFixtures.SampleCount; i++)
        {
            if (Fixed64.TryMultiplyDivide(
                s_rescuedMultiplyDivideValue,
                s_rescuedMultiplyDivideValue,
                Fixed64.One,
                s_rescuedMultiplyDivideValue,
                out Fixed64 result))
            {
                accumulator += result;
            }
        }

        return accumulator;
    }

    [Benchmark]
    public Fixed64 FastDiv()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += FixedMath.FastDiv(_left[i], _positive[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 FastMulFractionalOperands()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += FixedMath.FastMul(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 FastMulIntegerLeftOperand()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += FixedMath.FastMul(_integerScalars[i], _left[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 FastMulIntegerRightOperand()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += FixedMath.FastMul(_left[i], _integerScalars[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Sqrt()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _positive.Length; i++)
            accumulator += FixedMath.Sqrt(_positive[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Sin()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _angles.Length; i++)
            accumulator += FixedMath.Sin(_angles[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Cos()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _angles.Length; i++)
            accumulator += FixedMath.Cos(_angles[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 DegToRad()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _degreeAngles.Length; i++)
            accumulator += FixedMath.DegToRad(_degreeAngles[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 RadToDeg()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _angles.Length; i++)
            accumulator += FixedMath.RadToDeg(_angles[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Tan()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _tangentAngles.Length; i++)
            accumulator += FixedMath.Tan(_tangentAngles[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Acos()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _unit.Length; i++)
            accumulator += FixedMath.Acos(_unit[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Asin()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _unit.Length; i++)
            accumulator += FixedMath.Asin(_unit[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Atan()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += FixedMath.Atan(_left[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Atan2()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _left.Length; i++)
            accumulator += FixedMath.Atan2(_left[i], _right[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Pow()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _positive.Length; i++)
            accumulator += FixedMath.Pow(_positive[i], _unit[i] * Fixed64.Half);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Pow2()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _unit.Length; i++)
            accumulator += FixedMath.Pow2(_unit[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Log2()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _positive.Length; i++)
            accumulator += FixedMath.Log2(_positive[i]);

        return accumulator;
    }

    [Benchmark]
    public Fixed64 Ln()
    {
        Fixed64 accumulator = Fixed64.Zero;
        for (int i = 0; i < _positive.Length; i++)
            accumulator += FixedMath.Ln(_positive[i]);

        return accumulator;
    }
}
