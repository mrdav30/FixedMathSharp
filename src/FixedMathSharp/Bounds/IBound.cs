namespace FixedMathSharp
{
    public interface IBound
    {
        Vector3d Center { get; }
        bool Contains(Vector3d point);
        bool Intersects(IBound other);
        Fixed64 DistanceToSurface(Vector3d point);
    }
}