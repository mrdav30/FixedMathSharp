namespace FixedMathSharp.Editor
{
    // Union type representing either a property name or array element index.  The element
    // index is valid only if propertyName is null.
    public struct PropertyPathComponent
    {
        public string propertyName;
        public int elementIndex;
    }
}