#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

namespace FixedMathSharp.Editor
{
    /// <summary>
    /// A custom property drawer for vectors type structures.
    /// </summary>
    /// <see cref="PropertyDrawer" />
    [CustomPropertyDrawer(typeof(Vector2))]
    [CustomPropertyDrawer(typeof(Vector2d))]
    [CustomPropertyDrawer(typeof(Vector3))]
    [CustomPropertyDrawer(typeof(Vector3d))]
    public class VectorDrawer : PropertyDrawer
    {
        /// <summary>
        /// Called when the GUI is drawn.
        /// </summary>
        /// <param name="position">Rectangle on the screen to use for the property GUI.</param>
        /// <param name="property">The SerializedProperty to make the custom GUI for.</param>
        /// <param name="label">The label of this property.</param>
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            int fieldCount = 3;
            Rect contentPosition = EditorGUI.PrefixLabel(position, label);

            EditorGUIUtility.labelWidth = 14f;
            float fieldWidth = contentPosition.width / fieldCount;
            bool hideLabels = contentPosition.width < 185;
            contentPosition.width /= fieldCount + 0.5f;

            using var indent = new EditorGUI.IndentLevelScope(-EditorGUI.indentLevel);
            for (int i = 0; i < fieldCount; i++)
            {
                if (!property.NextVisible(true))
                    break;

                EditorGUI.BeginProperty(contentPosition, label, property);
                {
                    EditorGUI.BeginChangeCheck();

                    // for vector3
                    if (property.type == "float")
                    {
                        float newVal = EditorGUI.FloatField(
                            contentPosition,
                            hideLabels ? GUIContent.none : new GUIContent(property.displayName),
                            property.floatValue);
                        if (EditorGUI.EndChangeCheck())
                            property.floatValue = newVal;
                    }
                    else if (property.type == "Fixed64")
                    {
                        string displayName = property.displayName;
                        // move to next property to get Fixed64.m_rawValue
                        if (!property.NextVisible(true))
                            break;

                        Fixed64 newVal = FixedMathEditorUtility.FixedNumberField(
                                        contentPosition,
                                        hideLabels ? GUIContent.none : new GUIContent(displayName),
                                        property.longValue);
                        if (EditorGUI.EndChangeCheck())
                            property.longValue = newVal.m_rawValue;
                    }

                }
                EditorGUI.EndProperty();

                contentPosition.x += fieldWidth;
            }
        }
    }
}
#endif