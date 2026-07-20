using FixedMathSharp.Bounds;
using System.Numerics;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    private static (Fixed64 Entry, Fixed64 Exit) GetRadialIntervalOracle(
        FixedSegment query,
        FixedSegment axis,
        Fixed64 radius)
    {
        BigInteger queryX = query.End.X.m_rawValue - (BigInteger)query.Start.X.m_rawValue;
        BigInteger queryY = query.End.Y.m_rawValue - (BigInteger)query.Start.Y.m_rawValue;
        BigInteger queryZ = query.End.Z.m_rawValue - (BigInteger)query.Start.Z.m_rawValue;
        BigInteger axisX = axis.End.X.m_rawValue - (BigInteger)axis.Start.X.m_rawValue;
        BigInteger axisY = axis.End.Y.m_rawValue - (BigInteger)axis.Start.Y.m_rawValue;
        BigInteger axisZ = axis.End.Z.m_rawValue - (BigInteger)axis.Start.Z.m_rawValue;
        BigInteger startX = query.Start.X.m_rawValue - (BigInteger)axis.Start.X.m_rawValue;
        BigInteger startY = query.Start.Y.m_rawValue - (BigInteger)axis.Start.Y.m_rawValue;
        BigInteger startZ = query.Start.Z.m_rawValue - (BigInteger)axis.Start.Z.m_rawValue;
        BigInteger queryLengthSquared = Dot(queryX, queryY, queryZ, queryX, queryY, queryZ);
        BigInteger axisLengthSquared = Dot(axisX, axisY, axisZ, axisX, axisY, axisZ);
        BigInteger directionsDot = Dot(queryX, queryY, queryZ, axisX, axisY, axisZ);
        BigInteger startDirectionProjection = Dot(startX, startY, startZ, queryX, queryY, queryZ);
        BigInteger startAxisProjection = Dot(startX, startY, startZ, axisX, axisY, axisZ);
        BigInteger startDistanceSquared = Dot(startX, startY, startZ, startX, startY, startZ);
        BigInteger coefficient = (queryLengthSquared * axisLengthSquared) - (directionsDot * directionsDot);
        BigInteger projection = (startDirectionProjection * axisLengthSquared)
            - (startAxisProjection * directionsDot);
        BigInteger radiusSquared = (BigInteger)radius.m_rawValue * radius.m_rawValue;
        BigInteger constant = (startDistanceSquared * axisLengthSquared)
            - (startAxisProjection * startAxisProjection)
            - (radiusSquared * axisLengthSquared);
        BigInteger scaledVertex = -projection * FixedMath.ONE_L;
        BigInteger vertexFloor = BigInteger.DivRem(scaledVertex, coefficient, out BigInteger vertexRemainder);
        BigInteger vertexCeiling = vertexRemainder.IsZero ? vertexFloor : vertexFloor + BigInteger.One;
        long vertexFloorRaw = (long)BigInteger.Max(
            BigInteger.Zero,
            BigInteger.Min(vertexFloor, FixedMath.ONE_L));
        long vertexCeilingRaw = (long)BigInteger.Max(
            BigInteger.Zero,
            BigInteger.Min(vertexCeiling, FixedMath.ONE_L));

        long lowerFloor = FindLowerRootFloor(coefficient, projection, constant, vertexCeilingRaw, out bool lowerExact);
        long upperFloor = FindUpperRootFloor(coefficient, projection, constant, vertexFloorRaw, out bool upperExact);
        return (
            RoundLowerRoot(coefficient, projection, constant, lowerFloor, lowerExact),
            RoundUpperRoot(coefficient, projection, constant, upperFloor, upperExact));
    }

    private static long FindLowerRootFloor(
        BigInteger coefficient,
        BigInteger projection,
        BigInteger constant,
        long highRaw,
        out bool exact)
    {
        long lowRaw = 0L;
        while (highRaw - lowRaw > 1L)
        {
            long middleRaw = lowRaw + ((highRaw - lowRaw) >> 1);
            if (EvaluateDerivative(coefficient, projection, middleRaw, FixedMath.ONE_L).Sign < 0
                && Evaluate(coefficient, projection, constant, middleRaw, FixedMath.ONE_L).Sign > 0)
            {
                lowRaw = middleRaw;
            }
            else
            {
                highRaw = middleRaw;
            }
        }

        exact = Evaluate(coefficient, projection, constant, highRaw, FixedMath.ONE_L).IsZero;
        return exact ? highRaw : lowRaw;
    }

    private static long FindUpperRootFloor(
        BigInteger coefficient,
        BigInteger projection,
        BigInteger constant,
        long lowRaw,
        out bool exact)
    {
        long highRaw = FixedMath.ONE_L;
        while (highRaw - lowRaw > 1L)
        {
            long middleRaw = lowRaw + ((highRaw - lowRaw) >> 1);
            if (EvaluateDerivative(coefficient, projection, middleRaw, FixedMath.ONE_L).Sign <= 0
                || Evaluate(coefficient, projection, constant, middleRaw, FixedMath.ONE_L).Sign <= 0)
            {
                lowRaw = middleRaw;
            }
            else
            {
                highRaw = middleRaw;
            }
        }

        exact = Evaluate(coefficient, projection, constant, lowRaw, FixedMath.ONE_L).IsZero;
        return lowRaw;
    }

    private static Fixed64 RoundLowerRoot(
        BigInteger coefficient,
        BigInteger projection,
        BigInteger constant,
        long floorRaw,
        bool exact)
    {
        if (exact)
            return Fixed64.FromRaw(floorRaw);

        long midpoint = (floorRaw * 2L) + 1L;
        BigInteger value = Evaluate(coefficient, projection, constant, midpoint, FixedMath.ONE_L * 2L);
        int derivative = EvaluateDerivative(coefficient, projection, midpoint, FixedMath.ONE_L * 2L).Sign;
        if (value.IsZero && derivative <= 0)
            return Fixed64.FromRaw((floorRaw & 1L) == 0L ? floorRaw : floorRaw + 1L);
        return Fixed64.FromRaw(derivative < 0 && value.Sign > 0 ? floorRaw + 1L : floorRaw);
    }

    private static Fixed64 RoundUpperRoot(
        BigInteger coefficient,
        BigInteger projection,
        BigInteger constant,
        long floorRaw,
        bool exact)
    {
        if (exact)
            return Fixed64.FromRaw(floorRaw);

        long midpoint = (floorRaw * 2L) + 1L;
        BigInteger value = Evaluate(coefficient, projection, constant, midpoint, FixedMath.ONE_L * 2L);
        int derivative = EvaluateDerivative(coefficient, projection, midpoint, FixedMath.ONE_L * 2L).Sign;
        if (value.IsZero && derivative >= 0)
            return Fixed64.FromRaw((floorRaw & 1L) == 0L ? floorRaw : floorRaw + 1L);
        return Fixed64.FromRaw(derivative <= 0 || value.Sign <= 0 ? floorRaw + 1L : floorRaw);
    }

    private static BigInteger Evaluate(
        BigInteger coefficient,
        BigInteger projection,
        BigInteger constant,
        long numerator,
        long denominator) =>
        (coefficient * numerator * numerator)
        + (projection * numerator * denominator * 2)
        + (constant * denominator * denominator);

    private static BigInteger EvaluateDerivative(
        BigInteger coefficient,
        BigInteger projection,
        long numerator,
        long denominator) =>
        (coefficient * numerator) + (projection * denominator);

    private static BigInteger Dot(
        BigInteger leftX,
        BigInteger leftY,
        BigInteger leftZ,
        BigInteger rightX,
        BigInteger rightY,
        BigInteger rightZ) =>
        (leftX * rightX) + (leftY * rightY) + (leftZ * rightZ);
}
