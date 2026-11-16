using FixedMathSharp.Geometry;

namespace FixedMathSharp.Tests;

internal readonly struct ConePlaneRayTestEvent
{
    internal readonly ConePlaneRayEventSource Source;
    internal readonly ConePlaneRayEvent Event;
    internal ConePlaneRayTestEvent(ConePlaneRayEventSource source, ConePlaneRayEvent item) { Source = source; Event = item; }
}
