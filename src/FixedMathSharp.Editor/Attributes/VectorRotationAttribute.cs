using UnityEngine;

namespace FixedMathSharp
{
    /// <summary>
    /// Attribute to represent a fixed number angle in degrees. 
    /// Can be used to annotate properties or fields, supporting optional time-scaling and a maximum angle.
    /// </summary>
    public class VectorRotationAttribute : PropertyAttribute
    {
        public double Timescale { get; private set; }

        public VectorRotationAttribute(double timescale = 1d)
        {
            Timescale = timescale;
        }
    }
}