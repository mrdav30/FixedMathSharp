#if UNITY_EDITOR
using System.Collections;
using System.Reflection;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;

namespace FixedMathSharp.Editor
{
    // Union type representing either a property name or array element index.  The element
    // index is valid only if propertyName is null.
    internal struct FixedPropertyPathComponent
    {
        public string propertyName;
        public int elementIndex;
    }

    /// <summary>
    /// Provide simple value get/set methods for SerializedProperty.
    /// </summary>
    internal static class FixedSerializedPropertyExtensions
    {
        private static readonly Regex arrayElementRegex = new(@"\GArray\.data\[(\d+)\]", RegexOptions.Compiled);

        internal static object GetFixedPropertyValue(this SerializedProperty property)
        {
            return GetTargetObjectOfProperty(property, out _);
        }

        internal static void SetFixedPropertyValue(this SerializedProperty property, object value)
        {
            Undo.RecordObject(property.serializedObject.targetObject, $"Set {property.name}");
            SetValueNoRecord(property, value);
            EditorUtility.SetDirty(property.serializedObject.targetObject);
            property.serializedObject.ApplyModifiedProperties();
        }

        private static void SetValueNoRecord(this SerializedProperty property, object value)
        {
            var container = GetTargetObjectOfProperty(property, out var deferredToken);
            if (container == null)
            {
                Debug.LogError("Container is null, unable to set value.");
                return;
            }

            Debug.Assert(!container.GetType().IsValueType, $"Cannot use SetValue on a struct (temporary). Change {container.GetType().Name} to a class or use a parent reference.");

            SetPathComponentValue(container, deferredToken, value);
        }

        private static bool NextPathComponent(string propertyPath, ref int index, out FixedPropertyPathComponent component)
        {
            component = new FixedPropertyPathComponent();

            if (index >= propertyPath.Length) return false;

            var arrayElementMatch = arrayElementRegex.Match(propertyPath, index);
            if (arrayElementMatch.Success)
            {
                index += arrayElementMatch.Length + 1; // Skip past next '.'
                component.elementIndex = int.Parse(arrayElementMatch.Groups[1].Value);
                return true;
            }

            int dot = propertyPath.IndexOf('.', index);
            if (dot == -1)
            {
                component.propertyName = propertyPath.Substring(index);
                index = propertyPath.Length;
            }
            else
            {
                component.propertyName = propertyPath.Substring(index, dot - index);
                index = dot + 1; // Skip past next '.'
            }

            return true;
        }

        private static object GetPathComponentValue(object container, FixedPropertyPathComponent component)
        {
            if (container == null) return null;

            if (component.propertyName == null)
                return ((IList)container)[component.elementIndex];
            else
                return GetMemberValue(container, component.propertyName);
        }

        private static void SetPathComponentValue(object container, FixedPropertyPathComponent component, object value)
        {
            if (container == null) return;

            if (component.propertyName == null)
                ((IList)container)[component.elementIndex] = value;
            else
                SetMemberValue(container, component.propertyName, value);
        }

        private static object GetMemberValue(object container, string name)
        {
            if (container == null) return null;
            var type = container.GetType();
            var members = type.GetMember(name, BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Instance);
            foreach (var member in members)
            {
                if (member is FieldInfo field)
                    return field.GetValue(container);
                else if (member is PropertyInfo property)
                    return property.GetValue(container);
            }
            return null;
        }

        private static void SetMemberValue(object container, string name, object value)
        {
            if (container == null) return;

            var type = container.GetType();
            var members = type.GetMember(name, BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Instance);
            foreach (var member in members)
            {
                if (member is FieldInfo field)
                {
                    field.SetValue(container, value);
                    return;
                }
                else if (member is PropertyInfo property)
                {
                    property.SetValue(container, value);
                    return;
                }
            }
            Debug.Assert(false, $"Failed to set member {container}.{name} via reflection");
        }

        private static object GetTargetObjectOfProperty(SerializedProperty prop, out FixedPropertyPathComponent lastComponent, bool stopBeforeLast = false)
        {
            object obj = prop.serializedObject.targetObject;
            var path = prop.propertyPath.Replace(".Array.data[", "[");
            var elements = path.Split('.');

            lastComponent = new FixedPropertyPathComponent();
            for (int i = 0; i < elements.Length - (stopBeforeLast ? 1 : 0); i++)
            {
                if (NextPathComponent(elements[i], ref i, out var component))
                {
                    obj = GetPathComponentValue(obj, component);
                    lastComponent = component;
                }
            }
            return obj;
        }
    }
}
#endif