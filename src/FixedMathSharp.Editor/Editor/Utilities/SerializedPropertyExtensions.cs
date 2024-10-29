#if UNITY_EDITOR
using System;
using System.Collections;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;

namespace FixedMathSharp.Editor
{
    /// <summary>
    /// Provide simple value get/set methods for SerializedProperty.
    /// </summary>
    public static class SerializedPropertyExtensions
    {
        private static readonly Regex arrayElementRegex = new(@"\GArray\.data\[(\d+)\]", RegexOptions.Compiled);

        public static object? GetParent(SerializedProperty prop)
        {
            return GetTargetObjectOfProperty(prop, out _, true);
        }

        public static object? GetValue(this SerializedProperty property)
        {
            return GetTargetObjectOfProperty(property, out _);
        }

        public static object? GetValue(object? source, string name)
        {
            if (source == null) return null;

            var type = source.GetType();
            var field = type.GetField(name, BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Instance);
            if (field != null) return field.GetValue(source);

            var property = type.GetProperty(name, BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Instance | BindingFlags.IgnoreCase);
            return property?.GetValue(source);
        }

        public static object? GetValue(object? source, string name, int index)
        {
            if (source == null) return null;

            if (GetValue(source, name) is IEnumerable enumerable)
            {
                return enumerable.Cast<object?>().ElementAtOrDefault(index);
            }
            return null;
        }

        public static void SetValue(this SerializedProperty property, object value)
        {
            Undo.RecordObject(property.serializedObject.targetObject, $"Set {property.name}");
            SetValueNoRecord(property, value);
            EditorUtility.SetDirty(property.serializedObject.targetObject);
            property.serializedObject.ApplyModifiedProperties();
        }

        public static void SetValueNoRecord(this SerializedProperty property, object value)
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

        public static Type? GetPropertyType(this SerializedProperty property)
        {
            var propertyTarget = property.GetPropertyTarget();
            return propertyTarget?.GetType();
        }

        public static object? GetPropertyTarget(this SerializedProperty prop)
        {
            return GetTargetObjectOfProperty(prop, out _);
        }

        private static bool NextPathComponent(string propertyPath, ref int index, out PropertyPathComponent component)
        {
            component = new PropertyPathComponent();

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

        private static object? GetPathComponentValue(object? container, PropertyPathComponent component)
        {
            if (container == null) return null;

            if (component.propertyName == null)
                return ((IList)container)[component.elementIndex];
            else
                return GetMemberValue(container, component.propertyName);
        }

        private static void SetPathComponentValue(object? container, PropertyPathComponent component, object value)
        {
            if (container == null) return;

            if (component.propertyName == null)
                ((IList)container)[component.elementIndex] = value;
            else
                SetMemberValue(container, component.propertyName, value);
        }

        private static object? GetMemberValue(object? container, string name)
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

        private static void SetMemberValue(object? container, string name, object value)
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

        private static object? GetTargetObjectOfProperty(SerializedProperty prop, out PropertyPathComponent lastComponent, bool stopBeforeLast = false)
        {
            object? obj = prop.serializedObject.targetObject;
            var path = prop.propertyPath.Replace(".Array.data[", "[");
            var elements = path.Split('.');

            lastComponent = new PropertyPathComponent();
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