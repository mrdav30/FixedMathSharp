#if UNITY_EDITOR
using UnityEngine;

namespace FixedMathSharp
{
    public class FixedNumberAttribute : PropertyAttribute
    {
        public double Timescale { get; private set; }
        public bool Ranged { get; private set; }
        public long Min { get; private set; }
        public long Max { get; private set; }

        public FixedNumberAttribute(double timescale = 1d, bool ranged = false, long min = 0, long max = long.MaxValue)
        {
            Timescale = timescale;
            Ranged = ranged;
            Max = max;
            Min = min;
        }
    }
}
#endif