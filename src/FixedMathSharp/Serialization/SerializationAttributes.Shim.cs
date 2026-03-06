#if !NET8_0_OR_GREATER

// This file defines the attributes used for JSON serialization in .NET 8.0 and later versions.
// If the project is targeting an older version of .NET, these attributes are defined here to allow the code to compile without errors.
namespace System.Text.Json.Serialization.Shim;

internal sealed class JsonConverterAttribute : Attribute 
{
    public JsonConverterAttribute(Type converterType) { }
}
internal sealed class JsonIncludeAttribute : Attribute { }
internal sealed class JsonIgnoreAttribute : Attribute { }
internal sealed class JsonConstructorAttribute : Attribute { }
internal sealed class JsonPropertyNameAttribute : Attribute 
{
    public JsonPropertyNameAttribute(string name) { }
}

#endif