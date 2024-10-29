using UnityEngine;

namespace FixedMathSharp
{
    /// <summary>
    /// Attribute to represent a fixed number angle in degrees. 
    /// Can be used to annotate properties or fields, supporting optional time-scaling and a maximum angle.
    /// </summary>
    public class FixedNumberAngleAttribute : PropertyAttribute
    {
        /// <summary>
        /// Gets a value indicating whether the angle is scaled by time (e.g., frame rate).
        /// </summary>
        public double Timescale { get; private set; }

        /// <summary>
        /// Gets the maximum allowable value for the angle in degrees. 
        /// A value of -1 indicates no maximum limit.
        /// </summary>
        public double Max { get; private set; }

        /// <summary>
        /// Initializes a new instance of the <see cref="FixedNumberAngleAttribute"/> class.
        /// </summary>
        /// <param name="timescale">Specifies whether the angle should be scaled by time (e.g., frame rate).</param>
        /// <param name="max">The maximum allowable value for the angle in degrees. Default is -1 (no limit).</param>
        public FixedNumberAngleAttribute(double timescale = 1d, double max = -1d)
        {
            Timescale = timescale;
            Max = max;
        }
    }
}