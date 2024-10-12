#if UNITY_EDITOR || UNITY_5_3_OR_NEWER
using UnityEditor;
using UnityEngine;

namespace FixedMathSharp.Unity.Editor
{
    /// <summary>
    /// A custom property drawer for Fixed64 structures.
    /// </summary>
    /// <see also cref="PropertyDrawer" />\
    [CustomPropertyDrawer(typeof(FixedNumberAttribute))]
    [CustomPropertyDrawer(typeof(Fixed64))]
    public class Fixed64Drawer : PropertyDrawer
    {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            Rect contentPosition = EditorGUI.PrefixLabel(position, label);

            if (!property.NextVisible(true))
                return;

            EditorGUI.BeginProperty(contentPosition, label, property);
            {
                EditorGUI.BeginChangeCheck();

                Fixed64 newVal = FixedMathEditorUtility.FixedNumberField(
                                    contentPosition,
                                    GUIContent.none,
                                    property.longValue);

                if (EditorGUI.EndChangeCheck())
                {
                    property.longValue = newVal.RawValue;
                    property.serializedObject.ApplyModifiedProperties();
                }
            }
            EditorGUI.EndProperty();
        }
    }
}
#endif